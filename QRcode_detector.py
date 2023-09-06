import cv2
from pyzbar.pyzbar import decode
import numpy as np

# 이미지 로드
image = cv2.imread('qr_code_image.jpg')

# 필요시 웹캠 
# cap = cv2.VideoCapture(0)  # 뒤에 While ~~~ 필요

# 그레이스케일로 변환
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# QR 코드 검출 및 디코드
decoded_objects = decode(image)

for obj in decoded_objects:
    points = obj.polygon                # obj.polygon 의 return 값은 [(x1, y1), (x2, y2), (x3, y3), (x4, y4)] 이런식
    if len(points) > 4:
        hull = cv2.convexHull(np.array([point for point in points], dtype=np.float32))
        cv2.polylines(image, [hull], True, (0, 255, 0), 2)
    else:
        cv2.polylines(image, [np.array(points, dtype=np.int32)], True, (0, 255, 0), 2)

    barcode_data = obj.data.decode('utf-8')
    barcode_type = obj.type             # 대부분의 반환값은 QRCODE 라고 한다 이걸 이용해서 코드를 더 짜도 될듯
    text = f'Type: {barcode_type}, Data: {barcode_data}'
    cv2.putText(image, text, (obj.rect.left, obj.rect.bottom + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

# 결과 이미지 출력
cv2.imshow('QR Code Detection', image)
cv2.waitKey(0)
cv2.destroyAllWindows()
