#!/usr/bin/env python3
"""
Combine several PDFs into one and auto-crop the white margins.

Usage (example):
    python combine_and_crop.py \
        "week 1.pdf" "week 3.pdf" "week 8.pdf" \
        "week 12 - literature review.pdf" \
        "week 18.pdf" "week 25.pdf" \
        -o combined_weeks_cropped.pdf
"""

import argparse
from pathlib import Path
import pymupdf  # also imported as "fitz" in older examples


def crop_page_to_content(page, padding: float = 5.0):
    """
    Crop a single page to the bounding box of all visible objects,
    ensuring it stays within the page's physical MediaBox.
    """
    bboxlog = page.get_bboxlog()
    if not bboxlog:
        return  # nothing to crop

    xs0 = [b[1][0] for b in bboxlog]
    ys0 = [b[1][1] for b in bboxlog]
    xs1 = [b[1][2] for b in bboxlog]
    ys1 = [b[1][3] for b in bboxlog]

    min_x = min(xs0) - padding
    min_y = min(ys0) - padding
    max_x = max(xs1) + padding
    max_y = max(ys1) + padding

    rect = pymupdf.Rect(min_x, min_y, max_x, max_y)
    
    # --- THE FIX ---
    # Intersect the calculated rect with the physical page size (MediaBox)
    # to ensure we don't try to crop outside the page boundaries.
    rect = rect & page.mediabox 
    # ---------------

    page.set_cropbox(rect)

def combine_and_crop_pdfs(inputs, output, padding: float = 5.0):
    out_doc = pymupdf.open()  # empty PDF

    for pdf_path in inputs:
        src = pymupdf.open(pdf_path)

        # Crop all pages of this source PDF
        for page in src:
            crop_page_to_content(page, padding=padding)

        # Append the (now-cropped) pages to the output document
        out_doc.insert_pdf(src)
        src.close()

    out_doc.save(output)
    out_doc.close()


def main():
    parser = argparse.ArgumentParser(
        description="Combine PDFs and crop white margins from each page."
    )
    parser.add_argument("pdfs", nargs="+", help="Input PDF files in desired order.")
    parser.add_argument(
        "-o",
        "--output",
        default="combined_cropped.pdf",
        help="Output PDF filename (default: combined_cropped.pdf)",
    )
    parser.add_argument(
        "-p",
        "--padding",
        type=float,
        default=5.0,
        help="Padding (points) to keep around content (default: 5.0).",
    )

    args = parser.parse_args()

    input_files = [str(Path(p)) for p in args.pdfs]
    combine_and_crop_pdfs(input_files, args.output, padding=args.padding)


if __name__ == "__main__":
    main()
