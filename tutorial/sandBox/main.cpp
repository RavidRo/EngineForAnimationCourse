
#include "igl/opengl/glfw/renderer.h"
#include "tutorial/sandBox/inputManager.h"
#include "sandBox.h"

int main(int argc, char *argv[])
{
  Display *disp = new Display(1200, 800, "Wellcome");
  Renderer renderer;

  SandBox viewer(&renderer);
  
  igl::opengl::glfw::imgui::ImGuiMenu* menu = new igl::opengl::glfw::imgui::ImGuiMenu();
  viewer.Init("configuration.txt");
  
  Init(*disp, menu);
  renderer.init(&viewer,2,menu);
  //renderer.TranslateCamera(Eigen::Vector3d(0, 0, 0));
  disp->SetRenderer(&renderer);
  disp->launch_rendering(true);
  delete menu;
  delete disp;
}
