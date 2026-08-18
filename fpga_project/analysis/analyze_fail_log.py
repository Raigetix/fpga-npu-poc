#!/usr/bin/env python3
# analyze_fail_log.py -- analiza la salida de texto de sdram_freq_test.cpp
# (ver docs/caracterizacion-frecuencia-sdram.md) para distinguir entre las
# posibles causas de la corrupcion intermitente de la SDRAM:
#   1) defecto fisico localizado (una fila/celda mala) -- se verifica
#      comparando que direcciones fallan entre corridas independientes con
#      la misma semilla; un defecto real fallaria siempre en el mismo lugar.
#   2) "demasiadas lecturas seguidas" -- correlacion con la posicion/conteo
#      dentro de una pasada.
#   3) carrera de temporizacion entre operaciones consecutivas -- se detecta
#      si el byte "leido" coincide EXACTO con el "esperado" de una direccion
#      cercana (casi siempre la anterior), reconstruyendo la secuencia del
#      LFSR desde la semilla usada en la corrida.
#
# Uso: python analyze_fail_log.py <seed_hex> <addr_max> archivo1.txt [archivo2.txt ...]
#   (los .txt son la salida cruda de sdram_freq_test.cpp, con lineas
#   "addr=0x.. esperado=0x.. leido=0x.. pasada=..")
#
# Resultado obtenido la primera vez que se corrio esto (ver el doc de
# caracterizacion): 98.7% de las fallas en las 4 frecuencias eran
# exactamente el valor esperado de la direccion ANTERIOR -- descarta tanto
# "celda mala" (0% de repetibilidad de direccion entre corridas) como
# "desgaste por repeticion", y apunta a una carrera de temporizacion.
import re
import sys
from collections import Counter

LINE_RE = re.compile(r"addr=0x([0-9A-Fa-f]+) esperado=0x([0-9A-Fa-f]+) leido=0x([0-9A-Fa-f]+) pasada=(\d+)")


def lfsr_seq(seed, n):
    """Misma logica que sdram_test_harness.v: lfsr <= {lfsr[30:0], fb},
    fb = lfsr[31]^lfsr[21]^lfsr[1]^lfsr[0]. Devuelve lfsr[7:0] en cada paso."""
    out = []
    lfsr = seed
    for _ in range(n):
        out.append(lfsr & 0xFF)
        fb = ((lfsr >> 31) ^ (lfsr >> 21) ^ (lfsr >> 1) ^ lfsr) & 1
        lfsr = ((lfsr << 1) | fb) & 0xFFFFFFFF
    return out


def parse_entries(paths):
    entries = []
    for path in paths:
        with open(path, encoding="utf-8", errors="replace") as f:
            for line in f:
                m = LINE_RE.search(line)
                if m:
                    addr = int(m.group(1), 16)
                    esp = int(m.group(2), 16)
                    leido = int(m.group(3), 16)
                    entries.append((path, addr, esp, leido))
    return entries


def main():
    if len(sys.argv) < 4:
        print(__doc__)
        sys.exit(1)
    seed = int(sys.argv[1], 16)
    addr_max = int(sys.argv[2])
    paths = sys.argv[3:]

    expected = lfsr_seq(seed, addr_max)
    entries = parse_entries(paths)
    print(f"Total entradas: {len(entries)}")

    exact_minus1 = sum(1 for _, addr, esp, leido in entries if addr - 1 >= 0 and leido == expected[addr - 1])
    print(f"leido == esperado[addr-1] EXACTO: {exact_minus1} ({100*exact_minus1/max(1,len(entries)):.1f}%)")

    remaining = [(p, a, e, r) for p, a, e, r in entries if not (a - 1 >= 0 and r == expected[a - 1])]
    print(f"Restantes sin explicar por k=-1: {len(remaining)}")

    # Busca el |k| mas chico que explique cada resto, probando en orden
    # 1,-1,2,-2,... (mas cercano primero, no simplemente "el primero que
    # coincida en orden creciente de k", que sesgaria hacia k negativos).
    order = []
    for m in range(1, 41):
        order.append(m)
        order.append(-m)
    offset_counter = Counter()
    unexplained = 0
    for path, addr, esp, leido in remaining:
        found = None
        for k in order:
            a2 = addr + k
            if 0 <= a2 < addr_max and expected[a2] == leido:
                found = k
                break
        if found is not None:
            offset_counter[found] += 1
        else:
            unexplained += 1

    print(f"De los restantes: explicados por otro |k| chico: {sum(offset_counter.values())}, sin explicar: {unexplained}")
    print("Top offsets entre los restantes:")
    for k, c in offset_counter.most_common(10):
        print(f"    k={k:+4d}: {c}")

    # Repetibilidad de direcciones entre archivos (corridas) distintos --
    # si el mismo conjunto de direcciones falla siempre, sugiere defecto
    # fisico localizado en vez de carrera de temporizacion.
    by_file = {}
    for path, addr, esp, leido in entries:
        by_file.setdefault(path, set()).add(addr)
    if len(by_file) > 1:
        print("\nRepetibilidad de direcciones entre archivos:")
        all_sets = list(by_file.values())
        inter = set.intersection(*all_sets)
        union = set.union(*all_sets)
        for path, s in by_file.items():
            print(f"    {path}: {len(s)} direcciones distintas")
        print(f"    interseccion de TODOS: {len(inter)}  union: {len(union)}")


if __name__ == "__main__":
    main()
