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




# To Do
# 이 QR코드를 이용해서 거리나 방향을 확인하기 위해서는 cv2.minAreaRect 를 사용해야 할 수 있다. (비스듬하게 있는 사각형을 돌려주는 기능을 함)


# x, y, w, h = cv2.boundingRect(contour): 좌표를 감싸는 사각형 반환
# x, y: 사각형의 왼쪽 상단 좌표
# w, h: 사각형의 폭과 높이

# rotateRect = cv2.minAreaRect(contour): 좌표를 감싸는 최소한의 사각형 계산

# vertex = cv2.boxPoints(rotateRect): rotateRect로부터 꼭짓점 좌표 계산
# vertex: 4개의 꼭짓점 좌표, 소수점 포함이므로 정수 변환 필요

# center, radius = cv2.minEnclosingCircle(contour): 좌표를 감싸는 최소한의 동그라미 계산
# center: 원점 좌표(x, y)
# radius: 반지름

# area, triangle = cv2.minEnclosingTriangle(points): 좌표를 감싸는 최소한의 삼각형 게산
# area: 넓이
# triangle: 3개의 꼭짓점 좌표

# ellipse = cv2.fitEllipse(points): 좌표를 감싸는 최소한의 타원 계산

# line = cv2.fitLine(points, distType, param, reps, aeps, line): 중심점을 통과하는 직선 계산
# distType: 거리 계산 방식 (cv2.DIST_L2, cv2.DIST_L1, cv2.DIST_L12, cv2.DIST_FAIR, cv2.DIST_WELSCH, cv2.DIST_HUBER)
# param: distType에 전달할 인자, 0 = 최적 값 선택
# reps: 반지름 정확도, 선과 원본 좌표의 거리, 0.01 권장
# aeps: 각도 정확도, 0.01 권장
# line(optional): vx, vy 정규화된 단위 벡터, x0, y0: 중심점 좌표

# 출처 https://bkshin.tistory.com/entry/OpenCV-22-%EC%BB%A8%ED%88%AC%EC%96%B4Contour