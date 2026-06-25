"""
Generate a chessboard calibration target as a print-ready PDF.

OpenCV's checkerboard calibration wants a grid of black/white squares
of equal size; the function counts INNER corners, which is one less
than the number of squares along each side. We default to 10x7 squares
(so 9x6 inner corners), each 25 mm, which fits cleanly on A4 in
portrait with a sane margin.

When you print this PDF, choose "Actual size" / "100%" / "No scaling"
in the print dialog. After printing, measure one square with a ruler;
if it isn't exactly SQUARE_MM, pass the real value to
calibrate_camera.py — that's the number that turns into "real cm" in
the kalibration math.

Output: chessboard_a4_9x6.pdf in the same directory.
"""

from __future__ import annotations

from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.pdfgen import canvas


SQUARES_X = 10           # along the long side of the board
SQUARES_Y = 7            # along the short side
SQUARE_MM = 25.0         # each square is 25 mm × 25 mm
OUT_PATH = "chessboard_a4_9x6.pdf"


def main() -> None:
    page_w, page_h = A4
    board_w = SQUARES_X * SQUARE_MM * mm
    board_h = SQUARES_Y * SQUARE_MM * mm

    # Center the board on the page.
    margin_x = (page_w - board_w) / 2.0
    margin_y = (page_h - board_h) / 2.0

    c = canvas.Canvas(OUT_PATH, pagesize=A4)

    # Draw only the BLACK squares — white squares are just the paper
    # showing through. We start at top-left corner, count (row+col)
    # so the top-left square ends up black; OpenCV detects either
    # orientation but most tutorials show top-left-black so we match.
    c.setFillColorRGB(0, 0, 0)
    for row in range(SQUARES_Y):
        for col in range(SQUARES_X):
            if ((row + col) % 2) != 0:
                continue                     # skip white squares
            x = margin_x + col * SQUARE_MM * mm
            # PDF origin is bottom-left, so flip rows for top-down layout.
            y = margin_y + (SQUARES_Y - 1 - row) * SQUARE_MM * mm
            c.rect(x, y, SQUARE_MM * mm, SQUARE_MM * mm, stroke=0, fill=1)

    # Thick outline around the whole board so OpenCV's detector reliably
    # finds the outer edge (it expects a quiet zone between the pattern
    # and the paper edge, but a clear boundary helps a lot under uneven
    # lighting).
    c.setStrokeColorRGB(0, 0, 0)
    c.setLineWidth(1.5)
    c.rect(margin_x, margin_y, board_w, board_h, stroke=1, fill=0)

    # Footer with the spec — handy when there's a stack of test prints.
    c.setFillColorRGB(0, 0, 0)
    c.setFont("Helvetica", 9)
    label = (
        f"OpenCV chessboard | squares: {SQUARES_X}x{SQUARES_Y} "
        f"({SQUARES_X-1}x{SQUARES_Y-1} inner corners) | "
        f"square: {SQUARE_MM:.1f} mm | "
        "Print at 100% / Actual size"
    )
    c.drawString(margin_x, margin_y - 12, label)

    c.save()
    print(f"Wrote {OUT_PATH}: {SQUARES_X}x{SQUARES_Y} squares, "
          f"{SQUARE_MM:.1f} mm each. Inner corners = {SQUARES_X-1}x{SQUARES_Y-1}.")


if __name__ == "__main__":
    main()
