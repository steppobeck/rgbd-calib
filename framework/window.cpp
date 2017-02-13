#include "window.hpp"

#include <glm/gtc/type_ptr.hpp>

#include <cstring>
#include <iostream>

Window::Window(glm::ivec2 const& windowsize, bool mode3D)
  : m_mode3D(mode3D)
  , m_window(nullptr)
  , m_size(windowsize)
  , m_title("rgbd-calib")
  , m_mousePosition()
  , m_last_mousePosition()
  , m_mouseButtonFlags(0)
  , m_keypressed()
  , m_yaw(0.0)
  , m_pitch(0.0)
  , m_zoom(5.0)
  , m_camera_height(0.0)
  , m_camera_pan(0.0)
  , m_clear_color(0.0,0.0,0.0)
  , m_line_width(2.0)
{
  std::fill(std::begin(m_keypressed), std::end(m_keypressed), 0);
  glfwInit();
  glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_API);
  glfwWindowHint(GLFW_RESIZABLE, 0);
  m_window = glfwCreateWindow(windowsize.x, windowsize.y, m_title.c_str(), nullptr, nullptr);

  if (m_window) {
    glfwSetWindowUserPointer(m_window, this);
    assert(m_window != nullptr);

    glfwSetMouseButtonCallback(m_window, Window::mouseButtonCallback);
    glfwSetCursorPosCallback(m_window, Window::cursorPositionCallback);
    glfwSetKeyCallback(m_window, Window::keyCallback);
    glfwMakeContextCurrent(m_window);

    //glewExperimental = GL_TRUE;
    glewInit();
    //glGetError();

    //glEnable(GL_ALPHA_TEST);
    //glAlphaFunc(GL_NOTEQUAL, 0);
    //glEnable(GL_BLEND);
    //glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    //glEnable(GL_POINT_SMOOTH);
    glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
    //glPointSize(3.0f);
    glEnable(GL_POINT_SMOOTH);

    //glLineWidth(2.0f);
    //glEnable(GL_LINE_SMOOTH);
    
  }
}

Window::~Window()
{
  if (m_window) {
    glfwDestroyWindow(m_window);
    m_window = nullptr;
  }
  glfwTerminate();
}

void Window::cursorPositionCallback(GLFWwindow* win, double x, double y)
{
  Window* w = reinterpret_cast<Window*>(glfwGetWindowUserPointer(win));
  assert(w);

  w->m_mousePosition = glm::ivec2(x, y);
}

void Window::mouseButtonCallback(GLFWwindow* win, int button, int act, int mods)
{
  Window* w = reinterpret_cast<Window*>(glfwGetWindowUserPointer(win));
  assert(w);

  if (GLFW_PRESS == act) {
      switch (button) {
        case GLFW_MOUSE_BUTTON_LEFT:
          w->m_mouseButtonFlags |= Window::MOUSE_BUTTON_LEFT;
          break;
        case GLFW_MOUSE_BUTTON_MIDDLE:
          w->m_mouseButtonFlags |= Window::MOUSE_BUTTON_MIDDLE;
          break;
        case GLFW_MOUSE_BUTTON_RIGHT:
          w->m_mouseButtonFlags |= Window::MOUSE_BUTTON_RIGHT;
          break;
        default:
          break;
      }
  } else if (act == GLFW_RELEASE) {
    switch (button) {
      case GLFW_MOUSE_BUTTON_LEFT:
        w->m_mouseButtonFlags &= ~Window::MOUSE_BUTTON_LEFT;
        break;
      case GLFW_MOUSE_BUTTON_MIDDLE:
        w->m_mouseButtonFlags &= ~Window::MOUSE_BUTTON_MIDDLE;
        break;
      case GLFW_MOUSE_BUTTON_RIGHT:
        w->m_mouseButtonFlags &= ~Window::MOUSE_BUTTON_RIGHT;
        break;
      default:
        break;
    }
  }
}

void Window::keyCallback(GLFWwindow* win, int key, int scancode, int act, int mods)
{
  Window* w = reinterpret_cast<Window*>(glfwGetWindowUserPointer(win));
  assert(w);
  w->m_keypressed[key] = act == KEY_PRESS;
}

bool Window::shouldClose() const
{
  return glfwWindowShouldClose(m_window);
}

glm::vec2 Window::mousePosition() const
{
  return glm::vec2(m_mousePosition.x/float(m_size.x)
         , 1.0f - m_mousePosition.y/float(m_size.y));
}

void Window::stop()
{
  glfwSetWindowShouldClose(m_window, GL_TRUE);
}

void Window::update()
{
  glfwSwapBuffers(m_window);
  glfwPollEvents();

  // prepare next frame
  glViewport(0, 0, m_size.x, m_size.y);
  glClearColor(m_clear_color[0],m_clear_color[1],m_clear_color[2],0.0f);
  
  if(m_mode3D){

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glm::mat4 pers = glm::perspective(/*fovy*/45.0f, /*aspect*/ (1.0f*m_size.x)/m_size.y,
				      /*zNear*/0.01f, /*zFar*/50.0f);
    
    glMultMatrixf(glm::value_ptr(pers));
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    

    // ------------------------------ begin simple camera stuff
    static bool start_interacting_zoom = true;
    if(m_mouseButtonFlags & Window::MOUSE_BUTTON_RIGHT){
      if(start_interacting_zoom){
	m_last_mousePosition = m_mousePosition;
      }
      start_interacting_zoom = false;
      m_zoom -= 0.05 * (m_mousePosition.y * 1.0 - m_last_mousePosition.y);
      m_zoom = std::max(0.1f, m_zoom);
      m_last_mousePosition = m_mousePosition;
    }
    else{
      start_interacting_zoom = true;
    }

    glTranslatef(m_camera_pan, m_camera_height, -m_zoom);

    static bool start_interacting_rot = true;
    if(m_mouseButtonFlags & Window::MOUSE_BUTTON_LEFT){
      if(start_interacting_rot){
	m_last_mousePosition = m_mousePosition;
      }
      start_interacting_rot = false;
      m_yaw += 0.5 * (m_mousePosition.x * 1.0 - m_last_mousePosition.x);
      m_pitch += 0.5 * (m_mousePosition.y * 1.0 - m_last_mousePosition.y);
      m_last_mousePosition = m_mousePosition;
    }
    else{
      start_interacting_rot = true;
    }
    glRotatef(m_yaw, 0.0,1.0,0.0);
    glRotatef(-m_pitch, 1.0,0.0,0.0);

    // ------------------------------ end simple camera stuff

    


    drawCoords3D();


  }
  else{
    glClear(GL_COLOR_BUFFER_BIT);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0.0,1.0,0.0,1.0,0.01,100.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslatef(0.0, 0.0, -100.0);
  }
}

glm::ivec2 Window::windowSize() const
{
  glm::ivec2 size(0);
  glfwGetFramebufferSize(m_window, &size.x, &size.y);
  return size;
}

void Window::drawLine(glm::vec2 const& start
                    , glm::vec2 const& end
                    , glm::vec3 const& col) const
{
  glColor3f(GLfloat(col.r), GLfloat(col.g), GLfloat(col.b));
  glBegin(GL_LINES);
  {
    glVertex2f(GLfloat(start.x), GLfloat(start.y));
    glVertex2f(GLfloat(end.x), GLfloat(end.y));
  }
  glEnd();
}

void Window::drawLine(float startX, float startY,
                float endX, float endY,
                float r, float g, float b
                ) const
{

  drawLine(glm::vec2(startX, startY), glm::vec2(endX, endY), glm::vec3(r,g,b));
}

void Window::drawPoint(glm::vec2 const& p, glm::vec3 const& col) const
{
  glColor3f(GLfloat(col.r), GLfloat(col.g), GLfloat(col.b));
  glBegin(GL_POINTS);
    glVertex2f(GLfloat(p.x), GLfloat(p.y));
  glEnd();
}


void Window::drawPoint(float x, float y, float r, float g, float b) const
{
  drawPoint(glm::vec2(x,y), glm::vec3(r,g,b));
}

void Window::drawPoint(float x, float y, float z, float r, float g, float b) const
{
  glColor3f(GLfloat(r), GLfloat(g), GLfloat(b));
  glBegin(GL_POINTS);
  glVertex3f(GLfloat(x), GLfloat(y), GLfloat(z));
  glEnd();
}

float Window::getTime() const
{
  return float(glfwGetTime());
}


void
Window::drawCoords3D(){

  glLineWidth(m_line_width);
  glBegin(GL_LINES);
  glColor3f(1.0,0.0,0.0);
  glVertex3f(0.0,0.0,0.0);
  glVertex3f(1.0,0.0,0.0);
  for(unsigned i = 1; i < 10; ++i){
    glVertex3f(i * 0.1, i % 10 == 0 ? -0.05 : -0.02,0.0);
    glVertex3f(i * 0.1, i % 10 == 0 ? 0.05 : 0.02,0.0);
  }

  glColor3f(0.0,1.0,0.0);
  glVertex3f(0.0,0.0,0.0);
  glVertex3f(0.0,1.0,0.0);
  for(unsigned i = 1; i < 10; ++i){
    glVertex3f(i % 10 == 0 ? -0.05 : -0.02, i * 0.1, 0.0);
    glVertex3f(i % 10 == 0 ?  0.05 :  0.02, i * 0.1, 0.0);
  }

  glColor3f(0.0,0.0,1.0);
  glVertex3f(0.0,0.0,0.0);
  glVertex3f(0.0,0.0,1.0);
  for(unsigned i = 1; i < 10; ++i){
    glVertex3f(0.0,i % 10 == 0 ? -0.05 : -0.02, i * 0.1);
    glVertex3f(0.0,i % 10 == 0 ? 0.05 : 0.02, i * 0.1);
  }
  glEnd();
}

void
Window::drawCross3D(float x, float y, float z, float radius){
  glLineWidth(m_line_width);
  glColor3f(1.0,0.5,0.0);
  glBegin(GL_LINES);
  glVertex3f(x - 0.5 * radius,y               ,z);
  glVertex3f(x + 0.5 * radius,y               ,z);
  glVertex3f(x,               y - 0.5 * radius,z);
  glVertex3f(x,               y + 0.5 * radius,z);
  glEnd();
}

void
Window::setClearColor(float r, float g, float b){
  m_clear_color[0] = r;
  m_clear_color[1] = g;
  m_clear_color[2] = b;
}


void
Window::setCameraPosition(float camera_pan, float camera_height, float camera_zoom, float pitch, float yaw){
  m_camera_pan = camera_pan;
  m_camera_height = camera_height;
  m_zoom = camera_zoom;
  m_pitch = pitch;
  m_yaw = yaw;
}
