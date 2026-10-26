"""Excel bill of materials for a VR3 quote from the drawing's cut list CSV.

VR3 asks for, per part: part number, quantity, outside dimension, tube type (RD/SQ),
wall thickness, approximate length and any features (bends, holes ...). The CSV comes
from the native `vr3-package` command (weldment cut list table, lengths in inches).

  python scripts/vr3_bom.py cad/exports/<package>/<stem>-cutlist.csv
"""
from pathlib import Path
import csv, json, re, sys
from openpyxl import Workbook
from openpyxl.styles import Font, Alignment

BENT = {'MainHoop_1', 'FrontHoop_1'}  # supplied formed roll hoops


def main(csv_path):
    csv_path = Path(csv_path)
    rows = list(csv.DictReader(csv_path.open(encoding='utf-8-sig')))
    numbers = json.loads(csv_path.with_name(csv_path.name.replace('-cutlist.csv', '-partnumbers.json')).read_text(encoding='utf-8-sig'))
    tube_of = {n['partNumber']: n['tube'] for n in numbers}
    # Every row must carry the part's own numbering: its description and length must be those of
    # the cut-list item with that PARTNUMBER, and every part number must appear once. (The 12 Oct
    # 009c package's PART NO. column held item numbers, which mislabelled every BOM row.)
    item = {n['partNumber']: n for n in numbers}
    bad = [r['PART NO.'] for r in rows if r['PART NO.'] not in item or item[r['PART NO.']]['description'] != r['DESCRIPTION'] or item[r['PART NO.']]['length'] != r['LENGTH']]
    if bad or sorted(r['PART NO.'] for r in rows) != sorted(item):
        sys.exit(f'{csv_path.name}: cut list rows do not match the part numbers ({len(bad)} mismatched, e.g. {bad[:5]})')
    wb = Workbook(); ws = wb.active; ws.title = 'VR3 BOM'
    head = ['Part #', 'Qty', 'Tube OD (in)', 'Type', 'Wall (in)', 'Approx. length (in)', 'Material', 'Features / notes', 'Frame member']
    ws.append(head)
    for c in ws[1]: c.font = Font(bold=True); c.alignment = Alignment(horizontal='center')
    for r in rows:
        desc = r['DESCRIPTION']
        m = re.search(r'(\d+\.\d+)\s*(SQ)?\s*x\s*(\d*\.\d+)', desc, re.I)
        od, sq, wall = (m[1], bool(m[2]), m[3]) if m else ('', False, '')
        pn = r['PART NO.']; tube = tube_of.get(pn, '')
        notes = 'Formed roll hoop - bends per hoop sketch (CLR 3.25 in die)' if tube in BENT else ''
        ws.append([pn, int(r['QTY.']), float(od) if od else '', 'SQ' if sq else 'RD', float(wall) if wall else '',
                   float(r['LENGTH']), '4130N', notes, tube])
    widths = [8, 6, 13, 6, 10, 19, 9, 52, 30]
    for i, w in enumerate(widths): ws.column_dimensions[chr(65 + i)].width = w
    out = csv_path.with_name(csv_path.name.replace('-cutlist.csv', '-VR3-BOM.xlsx'))
    wb.save(out); print(out)


if __name__ == '__main__':
    main(sys.argv[1])
