// sdram_model.v -- modelo de comportamiento de una SDR SDRAM generica
// (2K filas x 256 columnas x 4 bancos x 32 bits), SOLO para simulacion.
// No modela tiempos reales del chip -- solo el PROTOCOLO de comandos
// (activar/leer/escribir/precargar/autorefresco) con temporizacion en
// ciclos de clk_sdram. Tiene asserts que abortan la simulacion si el
// controlador viola el protocolo (fila no abierta, etc.) -- eso es
// justamente lo que estamos buscando.
module sdram_model #(
    parameter ROW_WIDTH = 11,
    parameter COL_WIDTH = 8,
    parameter BANK_WIDTH = 2,
    parameter CAS = 2
)(
    inout  [31:0] SDRAM_DQ,
    input  [10:0] SDRAM_A,
    input  [1:0]  SDRAM_BA,
    input         SDRAM_nCS,
    input         SDRAM_nWE,
    input         SDRAM_nRAS,
    input         SDRAM_nCAS,
    input         SDRAM_CLK,
    input         SDRAM_CKE,
    input  [3:0]  SDRAM_DQM
);
    localparam NBANKS = 1 << BANK_WIDTH;
    localparam NROWS  = 1 << ROW_WIDTH;
    localparam NCOLS  = 1 << COL_WIDTH;

    // memoria: [banco][fila][columna] -> palabra de 32 bits
    reg [31:0] mem [0:NBANKS*NROWS*NCOLS-1];

    reg row_open [0:NBANKS-1];
    reg [ROW_WIDTH-1:0] open_row [0:NBANKS-1];

    reg [31:0] dq_out;
    reg        dq_oen;
    reg        trace_en = 1'b0;   // el testbench lo prende/apaga por referencia jerarquica
    assign SDRAM_DQ = dq_oen ? dq_out : 32'bz;

    // pipeline de lectura pendiente (latencia CAS): CAS etapas de registro
    // entre "se emitio READ" y "el dato sale valido", para que quede listo
    // justo en el ciclo T_RCD+CAS+1 que espera sdram.v (ver su comentario).
    reg [CAS-1:0] rd_pending;
    reg [31:0]  rd_addr_pipe [0:CAS-1];

    integer i;
    initial begin
        for (i = 0; i < NBANKS; i = i + 1) row_open[i] = 1'b0;
        rd_pending = 0;
        dq_oen = 1'b0;
    end

    function [31:0] flat_addr;
        input [BANK_WIDTH-1:0] bank;
        input [ROW_WIDTH-1:0]  row;
        input [COL_WIDTH-1:0]  col;
        begin
            flat_addr = (bank * NROWS + row) * NCOLS + col;
        end
    endfunction

    // decodificacion de comando por {RAS,CAS,WE}
    wire [2:0] cmd = {SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE};
    localparam CMD_SetModeReg  = 3'b000;
    localparam CMD_AutoRefresh = 3'b001;
    localparam CMD_PreCharge   = 3'b010;
    localparam CMD_BankActivate= 3'b011;
    localparam CMD_Write       = 3'b100;
    localparam CMD_Read        = 3'b101;
    localparam CMD_NOP         = 3'b111;

    always @(posedge SDRAM_CLK) begin
        dq_oen <= 1'b0;

        // pipeline de lectura: desplazar y sacar dato cuando llega al final
        if (rd_pending[0]) begin
            dq_out <= mem[rd_addr_pipe[0]];
            dq_oen <= 1'b1;
            if (trace_en) $display("[%0t]   <- entrego flat_addr=%0d valor=%08h", $time, rd_addr_pipe[0], mem[rd_addr_pipe[0]]);
        end
        for (i = 0; i < CAS-1; i = i + 1) begin
            rd_pending[i]    <= rd_pending[i+1];
            rd_addr_pipe[i]  <= rd_addr_pipe[i+1];
        end
        rd_pending[CAS-1] <= 1'b0;

        if (trace_en && cmd != CMD_NOP)
            $display("[%0t] CMD=%b BA=%0d A=%03h (col=%0d row_abierta=%0d)", $time, cmd, SDRAM_BA, SDRAM_A, SDRAM_A[COL_WIDTH-1:0], row_open[SDRAM_BA]);

        case (cmd)
            CMD_BankActivate: begin
                if (row_open[SDRAM_BA])
                    $display("[%0t] MODELO SDRAM: ERROR fila ya abierta en banco %0d (se esperaba PRECHARGE antes de ACTIVATE)", $time, SDRAM_BA);
                row_open[SDRAM_BA] <= 1'b1;
                open_row[SDRAM_BA] <= SDRAM_A[ROW_WIDTH-1:0];
            end
            CMD_Read: begin
                if (!row_open[SDRAM_BA])
                    $display("[%0t] MODELO SDRAM: ERROR lectura sin fila abierta en banco %0d", $time, SDRAM_BA);
                rd_pending[CAS-1]   <= 1'b1;
                rd_addr_pipe[CAS-1] <= flat_addr(SDRAM_BA, open_row[SDRAM_BA], SDRAM_A[COL_WIDTH-1:0]);
                // auto-precharge (A10=1): cierra la fila al terminar esta lectura
                if (SDRAM_A[10]) row_open[SDRAM_BA] <= 1'b0;
            end
            CMD_Write: begin
                if (!row_open[SDRAM_BA])
                    $display("[%0t] MODELO SDRAM: ERROR escritura sin fila abierta en banco %0d", $time, SDRAM_BA);
                begin : wr_blk
                    reg [31:0] a, cur, nw;
                    a = flat_addr(SDRAM_BA, open_row[SDRAM_BA], SDRAM_A[COL_WIDTH-1:0]);
                    cur = mem[a];
                    nw = cur;
                    if (!SDRAM_DQM[0]) nw[7:0]   = SDRAM_DQ[7:0];
                    if (!SDRAM_DQM[1]) nw[15:8]  = SDRAM_DQ[15:8];
                    if (!SDRAM_DQM[2]) nw[23:16] = SDRAM_DQ[23:16];
                    if (!SDRAM_DQM[3]) nw[31:24] = SDRAM_DQ[31:24];
                    mem[a] = nw;
                    if (trace_en) $display("[%0t]   -> escribio flat_addr=%0d dqm=%b dq=%08h nuevo=%08h", $time, a, SDRAM_DQM, SDRAM_DQ, nw);
                end
                if (SDRAM_A[10]) row_open[SDRAM_BA] <= 1'b0;
            end
            CMD_PreCharge: begin
                if (SDRAM_A[10]) begin
                    for (i = 0; i < NBANKS; i = i + 1) row_open[i] <= 1'b0;
                end else begin
                    row_open[SDRAM_BA] <= 1'b0;
                end
            end
            CMD_AutoRefresh: begin
                if (row_open[0] || row_open[1] || row_open[2] || row_open[3])
                    $display("[%0t] MODELO SDRAM: ERROR autorefresco con alguna fila todavia abierta", $time);
            end
            default: ;
        endcase
    end
endmodule
