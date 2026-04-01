#ifndef PANGOLIN_VISUALIZER_HPP
#define PANGOLIN_VISUALIZER_HPP

#include <pangolin/pangolin.h>

#include <Eigen/Core>
#include <string>

class PangolinVisualizer {
public:
  PangolinVisualizer(int width, int height) {
    pangolin::CreateWindowAndBind("Nav Viewer", width, height);
    glEnable(GL_DEPTH_TEST);

    s_cam = pangolin::OpenGlRenderState(
      pangolin::ProjectionMatrix(width, height, 420, 420, width/2, height/2, 0.1, 1000),
      pangolin::ModelViewLookAt(0, -5, -10, 0, 0, 0, pangolin::AxisY)
    );
    handler = new pangolin::Handler3D(s_cam);

    d_cam = &pangolin::CreateDisplay()
      .SetBounds(0, 1, 0, 1, -width/(float)height)
      .SetHandler(handler);
  }

  ~PangolinVisualizer() {
    delete handler;
  }

  void SetPose(const Eigen::Matrix4f& T) {
    T_wb = T;
  }

  void SetVelocity(const Eigen::Vector3f& v) {
    velocity = v;
  }

private:
  void DrawAxis(const std::string& name, float scale = 1.0f) {
    glLineWidth(3);
    glBegin(GL_LINES);

    // X - red
    glColor3f(1,0,0); glVertex3f(0,0,0); glVertex3f(scale,0,0);
    // Y - green
    glColor3f(0,1,0); glVertex3f(0,0,0); glVertex3f(0,scale,0);
    // Z - blue
    glColor3f(0,0,1); glVertex3f(0,0,0); glVertex3f(0,0,scale);
    glEnd();

    glColor3f(1.0, 1.0, 1.0);
    pangolin::GlFont::I().Text(name).Draw(scale * 0.1f, scale * 0.1f, scale * 0.1f);
  }

  void DrawVelocity() {
    glPushMatrix();
    glMultMatrixf(T_wb.data());

    glLineWidth(5);
    glColor3f(1,1,0);

    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(velocity[0], velocity[1], velocity[2]);
    glEnd();

    glPopMatrix();
  }

private:
  pangolin::OpenGlRenderState s_cam;
  pangolin::Handler3D* handler;
  pangolin::View* d_cam;

  Eigen::Matrix4f T_wb = Eigen::Matrix4f::Identity();
  Eigen::Vector3f velocity = Eigen::Vector3f::Zero();
};

#endif // PANGOLIN_VISUALIZER_HPP