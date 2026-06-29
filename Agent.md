# 프로젝트 컨텍스트: ESP32-IDF 기반 시스템 개발

이 파일은 AI 에이전트가 본 프로젝트의 구조, 스타일 가이드 및 기술적 의사결정 사항을 이해하기 위한 가이드라인입니다.

## 🛠 기술 스택
- **Framework:** ESP-IDF v5.4.3
- **Language:** C (C99/C11 표준 지향)
- **Build System:** CMake / idf.py. 루트의 env.bat를 먼저 실행하여 IDF 환경 설정을 반영해야 함 
- **Target Chip:** ESP32-기본

## 📂 프로젝트 구조
- `main/`: 메인 로직 및 `app_main.c` 위치
- `components/`: 재사용 가능한 사용자 정의 컴포넌트
- `managed_components/`: IDF Component Manager를 통해 추가된 라이브러리
- `sdkconfig`: ESP-IDF 구성 설정 파일

## 📝 코딩 스타일 및 규칙
1. **Naming Convention:**
    - 함수/변수: `snake_case` (예: `sensor_read_task`)
    - 컴포넌트 접두사: 함수명 앞에 컴포넌트 이름을 붙임 (예: `wifi_init_sta()`)
    - 상수/매크로: `UPPER_SNAKE_CASE`
2. **Error Handling:**
    - 모든 ESP-IDF API 호출 시 `esp_err_t` 리턴값을 확인하고 `ESP_ERROR_CHECK()` 또는 적절한 로그 처리를 수행할 것.
3. **Logging:**
    - `printf` 대신 `ESP_LOGI`, `ESP_LOGE`, `ESP_LOGW` 매크로를 사용할 것. (Tag는 파일 상단에 정의)
4. **FreeRTOS 활용:**
    - Task 생성 시 Stack Size와 Priority를 명시적으로 관리할 것.
    - 공유 자원 접근 시 `Mutex` 또는 `Semaphore` 필수 사용.

## 🔗 참조할 기존 코드 패턴 (Reference)
- **NVS 저장:** `components/storage/nvs_helper.c`의 패턴 사용
- **메모리 관리:** 정적 할당보다는 `pvPortMalloc` 또는 `heap_caps_malloc` 사용 선호

## ⚙️ 하드웨어 제어 규칙
- **DC 모터 구동:** 트랙 및 터렛 제어는 `MCPWM`을 사용함. 터렛 회전 속도 조절은 `my_flatform.c`의 `TURRET_SPEED` 매크로로 제어 (현재 최대 속도인 511로 설정됨).
- **서보 모터:** **`LEDC`**를 사용하여 제어할 것. 포신 반동(GPIO 32) 및 포 마운트(SG90, GPIO 13) 모두 LEDC 채널 0/1을 사용.

## ⚠️ 주의 사항
- `sdkconfig`를 직접 수정하지 말고, 설정 변경이 필요하면 `menuconfig` 항목을 언급해줄 것.
- IDF 5.4.3 버전의 API를 사용할 것. 다른 버전은 고려하지 않고 있음
- **하드웨어 핀(Pin) 맵 및 상세 기능 정의는 프로젝트 루트의 `README.md` 파일을 최우선으로 참조할 것.**
- **리코일 사양:** 서보 각도는 0(Rest) ~ 60(Pull) 범위를 사용하며, 트랙 후진 반동 시간은 200ms를 유지함.
- **사운드 피드백 제약:** 사운드 모듈의 RX 핀이 연결되지 않은 상태이므로, `rctank_dfplayer`의 자동 복구 기능 대신 `main` 로직의 타이머나 모듈 자체 루프 명령을 사용하여 상태를 동기화함.
