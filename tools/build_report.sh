#!/usr/bin/env bash
# Rebuild the Word report from docs/report.md (needs pandoc: sudo apt install pandoc).
#   ./tools/build_report.sh   -> docs/CS655_Q8_Report.docx
set -e
cd "$(dirname "$0")/../docs"
pandoc report.md -o CS655_Q8_Report.docx --resource-path=.
echo "Wrote docs/CS655_Q8_Report.docx"
