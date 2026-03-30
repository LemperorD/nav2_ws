#ifndef PANGOLIN_VISUALIZER_HPP
#define PANGOLIN_VISUALIZER_HPP

#include <pangolin/pangolin.h>

class PangolinVisualizer {

public:
  PangolinVisualizer(int width, int height) {
    // 初始化Pangolin窗口
    pangolin::CreateWindowAndBind("3D Visualization", width, height);
    glEnable(GL_DEPTH_TEST);
    
    // 设置相机
    s_cam = pangolin::OpenGlRenderState(
        pangolin::ProjectionMatrix(width, height, 420, 420, 320, 240, 0.1, 1000),
        pangolin::ModelViewLookAt(0.0, 0.0, -5.0, 0.0, 0.0, 0.0, pangolin::AxisY)
    );

    handler = new pangolin::Handler3D(s_cam);
  }

  ~PangolinVisualizer() {
    delete handler;
  }

private:
  pangolin::OpenGlRenderState s_cam;
  pangolin::Handler3D* handler;

}; // class PangolinVisualizer

#endif // PANGOLIN_VISUALIZER_HPP