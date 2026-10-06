#!/usr/bin/env python3
"""Genere docs/rapport_avancement.pdf a partir de docs/rapport_avancement.md.

Markdown -> HTML (module Python `markdown`) -> PDF (Chrome / Chromium en mode headless).

  python3 docs/build_pdf.py                       # cree docs/rapport_avancement.pdf
  python3 docs/build_pdf.py --copy ~/Documents/"project cv"/Rapport.pdf

Dependances : `sudo apt install python3-markdown` (ou pip) et google-chrome ou chromium.
"""

import argparse
import os
import shutil
import subprocess
import sys
import tempfile

DOCS = os.path.dirname(os.path.abspath(__file__))
SOURCE = os.path.join(DOCS, 'rapport_avancement.md')
OUTPUT = os.path.join(DOCS, 'rapport_avancement.pdf')

CSS = """
@page { size: A4; margin: 18mm 16mm; }
body { font-family: 'Noto Sans', 'DejaVu Sans', sans-serif; font-size: 10.5pt; line-height: 1.45; color: #1d1d1f; }
h1 { font-size: 19pt; border-bottom: 2px solid #2b6cb0; padding-bottom: 6px; color: #1a365d; }
h2 { font-size: 14pt; color: #2b6cb0; margin-top: 22px; break-after: avoid; }
h3 { font-size: 11.5pt; margin-top: 16px; break-after: avoid; }
h4 { font-size: 10.5pt; margin-top: 14px; break-after: avoid; }
table { border-collapse: collapse; width: 100%; margin: 8px 0 12px; font-size: 9.5pt; }
tr { break-inside: avoid; }
th { background: #ebf4ff; text-align: left; }
th, td { border: 1px solid #cbd5e0; padding: 4px 7px; vertical-align: top; overflow-wrap: break-word; }
code { background: #f1f1f1; padding: 1px 4px; border-radius: 3px; font-size: 9pt; }
td code, th code { overflow-wrap: break-word; white-space: normal; }
pre { background: #f6f8fa; border: 1px solid #ddd; border-radius: 4px; padding: 8px 10px; font-size: 8.5pt;
      line-height: 1.3; white-space: pre-wrap; break-inside: avoid; }
pre code { background: none; padding: 0; }
img { max-width: 70%; display: block; margin: 10px auto 4px; border: 1px solid #ccc; }
em { color: #555; }
hr { border: 0; border-top: 2px solid #2b6cb0; margin: 28px 0; break-after: page; }
"""


def find_chrome():
    for name in ('google-chrome', 'chromium', 'chromium-browser'):
        path = shutil.which(name)
        if path:
            return path
    sys.exit('Chrome ou Chromium introuvable.')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--copy', help='copier aussi le PDF vers ce chemin')
    args = parser.parse_args()

    try:
        import markdown
    except ImportError:
        sys.exit('Module markdown manquant : sudo apt install python3-markdown')

    with open(SOURCE, encoding='utf8') as f:
        body = markdown.markdown(f.read(), extensions=['tables', 'sane_lists', 'fenced_code'])
    # <base> : les images relatives (img/...) sont resolues depuis docs/
    html = (f'<!doctype html><html lang="fr"><head><meta charset="utf-8"><base href="file://{DOCS}/">'
            f"<title>Rapport d'avancement</title><style>{CSS}</style></head><body>{body}</body></html>")

    with tempfile.TemporaryDirectory() as tmp:
        page = os.path.join(tmp, 'rapport.html')
        with open(page, 'w', encoding='utf8') as f:
            f.write(html)
        subprocess.run([find_chrome(), '--headless=new', '--disable-gpu', '--no-pdf-header-footer',
                        '--allow-file-access-from-files', f'--print-to-pdf={OUTPUT}', f'file://{page}'],
                       check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    if os.path.getsize(OUTPUT) < 50_000:
        sys.exit(f'PDF suspect ({os.path.getsize(OUTPUT)} octets) : verifier la generation')
    print(f'PDF : {OUTPUT}')
    if args.copy:
        destination = os.path.expanduser(args.copy)
        shutil.copyfile(OUTPUT, destination)
        print(f'Copie : {destination}')


if __name__ == '__main__':
    main()
