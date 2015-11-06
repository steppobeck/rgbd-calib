#ifndef BUW_WINDOW_HPP
#define BUW_WINDOW_HPP

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#define GLM_FORCE_RADIANS
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <glm/gtx/multiple.hpp>
#include <glm/gtc/matrix_transform.hpp> 
#include <array>
#include <string>

class Window
{
public:
  Window(glm::ivec2 const& windowsize = glm::ivec2(640, 480), bool mode3D = false);
  ~Window();

  enum MouseButton
  {
    MOUSE_BUTTON_NONE   = 0,
    MOUSE_BUTTON_LEFT   = (1 << 0),
    MOUSE_BUTTON_RIGHT  = (1 << 1),
    MOUSE_BUTTON_MIDDLE = (1 << 2)
  };

  enum KeyAction
  {
    KEY_PRESS   = GLFW_PRESS,
    KEY_RELEASE = GLFW_RELEASE,
    KEY_REPEAT  = GLFW_REPEAT
  };

  void drawLine(glm::vec2 const& start,
                glm::vec2 const& end,
                glm::vec3 const& color
                ) const;

  void drawLine(float startX, float startY,
                float endX, float endY,
                float r, float g, float b
                ) const;


  void drawPoint(glm::vec2 const& p, glm::vec3 const& col) const;

  void drawPoint(float x, float y, float r, float g, float b) const;

  void drawPoint(float x, float y, float z, float r, float g, float b) const;

  glm::vec2 mousePosition() const;

  bool shouldClose() const;
  void stop();
  void update();
  inline bool isKeyPressed(int key) const { return m_keypressed[key]; }
  glm::ivec2 windowSize() const;
  float getTime() const;

private:
  bool m_mode3D;
  GLFWwindow* m_window;
  glm::ivec2 m_size;
  std::string const m_title;
  glm::vec2 m_mousePosition;
  glm::vec2 m_last_mousePosition;

  int m_mouseButtonFlags;
  std::array<bool, 512> m_keypressed;

  static void cursorPositionCallback(GLFWwindow* win, double x, double y);
  static void mouseButtonCallback(GLFWwindow* win, int button, int action, int mods);
  static void keyCallback(GLFWwindow* win, int key, int scancode, int action, int mods);

  void drawCoords3D();

  float m_yaw;
  float m_pitch;
  float m_zoom;
  

};

#endif // define BUW_WINDOW_HPP
