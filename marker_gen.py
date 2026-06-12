import cv2
import numpy as np

aruco = cv2.aruco
dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

marker_id = 0
marker_size = 500  # marker'ın kendi boyutu, pixel

# ArUco marker üret
marker_img = aruco.generateImageMarker(dictionary, marker_id, marker_size)

# EN İYİ DEĞERLER: 
# DICT_4X4 için toplam grid 6x6 birimdir (4x4 veri + 2 birim siyah çerçeve).
# 500 / 6 = 83.33 piksel (1 modül boyutu). 
# Güvenli ve standartlara uygun en iyi beyaz border tam 1 modül (84 piksel) olmalıdır.
white_border = 84  

# En dıştaki kesim çizgisinin beyaz border'dan çalmaması için en ideal kalınlık 1 pikseldir.
cut_line_thickness = 1  

# Beyaz border ekle
marker_with_border = cv2.copyMakeBorder(
    marker_img,
    white_border,
    white_border,
    white_border,
    white_border,
    cv2.BORDER_CONSTANT,
    value=255
)

# En dışa ince siyah kesim çizgisi ekle
final_img = marker_with_border.copy()
height, width = final_img.shape

cv2.rectangle(
    final_img,
    (0, 0),
    (width - 1, height - 1),
    color=0,
    thickness=cut_line_thickness
)

# Görseli kaydet (Toplam boyut: 668x668 piksel)
cv2.imwrite("robot_marker_id_0_optimal.png", final_img)
