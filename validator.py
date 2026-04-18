import re
from collections import defaultdict

FILE = r'c:/Users/PC/Desktop/MeuOdrive/ODrive/Firmware/Board/v3/Src/usbd_hid_if.c'

EXPECTED = {
    0x01: 3,   # JOYSTICK
    0x13: 52,  # TELEMETRY
    0x14: 6,   # COMMAND
    0x15: 8    # CONFIG RESP
}

TYPE_MAP = {
    0x81: "INPUT",
    0x91: "OUTPUT",
    0xB1: "FEATURE"
}

with open(FILE, 'r', encoding='utf-8', errors='replace') as f:
    content = f.read()

# extrai descriptor
start = content.find('HID_ReportDesc')
start = content.find('{', start) + 1
end = content.find('\n};', start)
body = content[start:end]

# limpa comentários
body = re.sub(r'/\*.*?\*/', '', body, flags=re.DOTALL)
body = re.sub(r'//.*', '', body)

tokens = re.findall(r'0x[0-9A-Fa-f]{2}', body)
data = [int(x, 16) for x in tokens]

reports = defaultdict(lambda: {
    "size": 0,
    "types": set()
})

current_id = None
report_size = None
report_count = None

i = 0
while i < len(data):
    b = data[i]

    # REPORT_ID
    if b == 0x85:
        current_id = data[i+1]
        i += 2
        continue

    # REPORT_SIZE
    if b == 0x75:
        report_size = data[i+1]
        i += 2
        continue

    # REPORT_COUNT
    if b == 0x95:
        report_count = data[i+1]
        i += 2
        continue

    # COLLECTION → reseta contexto local
    if b == 0xA1:
        report_size = None
        report_count = None
        i += 2
        continue

    # END_COLLECTION → idem
    if b == 0xC0:
        report_size = None
        report_count = None
        i += 1
        continue

    # INPUT / OUTPUT / FEATURE
    if b in TYPE_MAP:
        rid = current_id if current_id is not None else 0

        if report_size is not None and report_count is not None:
            size_bytes = (report_size * report_count) // 8
            reports[rid]["size"] += size_bytes
            reports[rid]["types"].add(TYPE_MAP[b])

        i += 2
        continue

    i += 1

# =========================
# RESULTADO
# =========================
print("\n=== REPORTS ===")
for rid in sorted(reports):
    r = reports[rid]
    types = ",".join(sorted(r["types"]))
    print(f"0x{rid:02X}: {r['size']} bytes [{types}]")

# =========================
# VALIDAÇÕES
# =========================

print("\n=== VALIDAÇÃO ===")

# 1. IDs duplicados entre tipos
for rid, r in reports.items():
    if len(r["types"]) > 1:
        print(f"ERRO: Report ID 0x{rid:02X} usado em múltiplos tipos: {r['types']}")

# 2. validar tamanhos esperados
for rid, expected_size in EXPECTED.items():
    real = reports.get(rid, {}).get("size")
    if real is None:
        print(f"ERRO: Report ID 0x{rid:02X} não encontrado")
    elif real != expected_size:
        print(f"ERRO: Report ID 0x{rid:02X} esperado {expected_size} mas encontrado {real}")
    else:
        print(f"OK: Report ID 0x{rid:02X} = {real} bytes")

# 3. detectar IDs duplicados no descriptor bruto
ids_raw = re.findall(r'0x85,\s*(0x[0-9A-Fa-f]{2})', body)
dupes = {x for x in ids_raw if ids_raw.count(x) > 1}
for d in dupes:
    print(f"ALERTA: ID duplicado no descriptor: {d}")