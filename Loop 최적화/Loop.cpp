// 1. Loop Unrolling
// 이점이 되는부분 
// 1. 루프의 조건검사, 인덱스증가와 같은 오버헤드를 줄여줍니다. 
// 2. 메모리의 prefetching의 효율을 증가시켜줍니다. 
//     - memory prefetching 이란  CPU가 미리 데이터를 캐시에 로드하여 메모리 접근 시간을 단축시키는 기법
//     - CPU나 메모리 컨트롤러는 접근 패턴을 분석하여 앞으로 접근 가능성이 높은 데이터를 미리 캐시에 로드하는데 이는 데이터가 연속된 메모리 주소에 있을때 효과적
//     - 이 예제에서는 array 의 값이 cache 데이터에 들어가 있으므로 (cache 데이터는 메모리에서 뺴오는것이 아니라 CPU에 더 가깝기 떄문에 속도면에서 엄청 빠르다) 언롤링을 한 루프가 더 속도가 빠름
//     - 하지만 Loop Unrolling 이 모든 상황에서 성능향상이 되는것이 아니며 사용 가능한 레지스터 수에 제약이 있을 경우 레지스터 압박이 될수이싿. (레지스터는 CPU에 달려있고 캐시는 CPU와 RAM 사이에 있음 크기는 레지스터가 더 작고 속도는 CPU와 가까운 레지스터가 더 빠름)



// 기본 루프
for (int i = 0; i < 100; i++) {
    array[i] = i * 2;
}

// 루프 언롤링 적용
for (int i = 0; i < 100; i += 5) {
    array[i] = i * 2;
    array[i+1] = (i+1) * 2;
    array[i+2] = (i+2) * 2;
    array[i+3] = (i+3) * 2;
    array[i+4] = (i+4) * 2;
}




//====================================================================================

// 2. Loop Fusion


// 루프 분리
for (int i = 0; i < N; i++) {
    arrayA[i] = i * 2;
}
for (int i = 0; i < N; i++) {
    arrayB[i] = i + 3;
}

// 루프 퓨전 적용
for (int i = 0; i < N; i++) {
    arrayA[i] = i * 2;
    arrayB[i] = i + 3;
}



//====================================================================================

// 3. Loop Hoisting

// 최적화 전
for (int i = 0; i < N; i++) {
    int temp = a * b;  // 변하지 않는 값
    array[i] = temp + i;
}

// 루프 호이스팅 적용
int temp = a * b;
for (int i = 0; i < N; i++) {
    array[i] = temp + i;
}





//====================================================================================

// 4. Loop Spliting

// 최적화 전
for (int i = 0; i < N; i++) {
    processA(array[i]);
    processB(array[i]);
}

// 루프 분할 적용
for (int i = 0; i < N; i++) {
    processA(array[i]);
}
for (int i = 0; i < N; i++) {
    processB(array[i]);
}











