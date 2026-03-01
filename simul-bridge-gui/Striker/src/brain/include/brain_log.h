#pragma once

#include <string>
#include <rerun.hpp>
#include "types.h"


class Brain; // 전방 선언

using namespace std;

/*
 * rerun log 관련 작업을 이 라이브러리에서 처리
 * 반복적으로 출력되는 로그가 있다면 이 클래스에 함수로 캡슐화
 */
class BrainLog
{
public:
    /**
     * 생성자에서는 객체만 초기화하고 enabled는 false로 설정
     */
    BrainLog(Brain *argBrain);

    // 필드 라인 등과 같은 정적 로그를 rerun log로 출력
    void prepare();

    // 필드 라인 등과 같은 정적 내용을 rerun log에 기록
    void logStatics();

    // 현재 시간 설정
    void setTimeNow();

    // 시간 수동 설정
    void setTimeSeconds(double seconds);

    // rerun::RecordingStream과 동일한 인터페이스 제공
    template <typename... Ts>
    inline void log(string_view entity_path, const Ts &...archetypes_or_collections) const
    {
        if (enable_log_tcp) {
            log_tcp.log(entity_path, archetypes_or_collections...);
        }

        if (enable_log_file) {
            log_file.log(entity_path, archetypes_or_collections...);
        }
    }

    // img log에서 정보를 직관적으로 표시하며 이미지 외곽에 padding 픽셀만큼 color 색상의 테두리를 그리고 하단에 text 표시
    void logToScreen(string logPath, string text, u_int32_t color, double padding = 0.0);

    void logRobot(string logPath, Pose2D pose, u_int32_t color, string label = "", bool draw2mCircle = false);

    void logBall(string logPath, Point pos, u_int32_t color, bool detected, bool known);

    rerun::LineStrip2D circle (float x, float y, float r, int nSeg = 36);
    rerun::LineStrip2D crosshair(float x, float y, float r);

    // 호출 시 로그를 새로운 파일로 stream하여 로그 파일이 과도하게 커지는 것을 방지
    void updateLogFilePath();

private:
    Brain *brain;
    bool enable_log_tcp = false;
    bool enable_log_file = false;
    rerun::RecordingStream log_tcp;
    rerun::RecordingStream log_file;
};