import re

with open(r'c:/Users/PC/Desktop/MeuOdrive/ODrive/Firmware/Board/v3/Src/usbd_hid_if.c', 'r', encoding='utf-8', errors='replace') as f:
    content = f.read()

# pega só o descriptor
start = content.find('HID_ReportDesc')
start = content.find('{', start) + 1
end = content.find('\n};', start)
body = content[start:end]

# limpa comentários
body = re.sub(r'/\*.*?\*/', '', body, flags=re.DOTALL)
body = re.sub(r'//.*', '', body)

# extrai bytes
tokens = re.findall(r'0x[0-9A-Fa-f]{2}', body)
data = [int(x, 16) for x in tokens]

reports = {}
current_id = None
report_size = None
report_count = None

i = 0
while i < len(data):
    b = data[i]

    # REPORT_ID
    if b == 0x85:
        current_id = data[i+1]
        reports.setdefault(current_id, 0)
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

    # INPUT / OUTPUT / FEATURE
    if b in (0x81, 0x91, 0xB1):
        if current_id is not None and report_size and report_count:
            size_bytes = (report_size * report_count) // 8
            reports[current_id] += size_bytes
        i += 2
        continue

    i += 1

# print resultado
for rid, size in sorted(reports.items()):
    print(f"Report ID 0x{rid:02X}: {size} bytes")