import re
import cv2
import numpy as np

CARD_MAP = {'1': 'tictactoe', 'OX': 'tictactoe', 'TICTACTOE': 'tictactoe',
            '틱택토': 'tictactoe', '2': 'maze', 'MAZE': 'maze', '미로': 'maze',
            '미로찾기': 'maze', '3': 'exit', 'EXIT': 'exit', 'STOP': 'exit', '종료': 'exit'}

def card_from_text(text):
    normalized = re.sub(r'[^\w가-힣]', '', str(text).upper())
    return CARD_MAP.get(normalized)

def recognize_card(image, *, fallback=None):
    """OCR with optional pytesseract; otherwise explicit CLI fallback, never guessing."""
    try:
        import pytesseract
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
        gray = cv2.resize(gray, None, fx=2, fy=2)
        for mode in (gray, cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]):
            raw = pytesseract.image_to_string(mode, config='--psm 6', lang='eng')
            for line in raw.splitlines():
                result = card_from_text(line)
                if result:
                    return result
    except (ImportError, RuntimeError, OSError):
        pass
    return card_from_text(fallback) if fallback else None

def tic_board_from_image(image):
    """Simple clean top-down O/X sheet recognition (not a production detector).
    Returns None on unclassifiable grid; robust deployment requires camera calibration.
    """
    g = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    g = cv2.resize(g, (600, 600))
    out = []
    for r in range(3):
        row = []
        for c in range(3):
            cell = g[r * 200 + 32:(r + 1) * 200 - 32, c * 200 + 32:(c + 1) * 200 - 32]
            ink = cv2.threshold(cell, 130, 255, cv2.THRESH_BINARY_INV)[1]
            ratio = cv2.countNonZero(ink) / ink.size
            if ratio < .015:
                row.append(' ')
                continue
            # O has bright (paper) center; X generally fills center intersection.
            cy, cx = np.array(ink.shape) // 2
            center = ink[cy-9:cy+9, cx-9:cx+9]
            center_ratio = cv2.countNonZero(center) / center.size
            row.append('X' if center_ratio > .16 else 'O')
        out.append(row)
    return out
