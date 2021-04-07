#include "GLViewer.hpp"

GLchar* FPC_VERTEX_SHADER =
    "#version 330 core\n"
    "layout(location = 0) in vec4 in_VertexRGBA;\n"
    "uniform mat4 u_mvpMatrix;\n"
    "uniform float pointsize;\n"
    "out vec3 b_color;\n"
    "void main() {\n"
    "   uint vertexColor = floatBitsToUint(in_VertexRGBA.w); \n"
    "   b_color = vec3(((vertexColor & uint(0x00FF0000)) >> 16) / 255.f, ((vertexColor & uint(0x0000FF00)) >> 8) / 255.f, (vertexColor & uint(0x000000FF)) / 255.f);\n"
    "	gl_Position = u_mvpMatrix * vec4(in_VertexRGBA.xyz, 1);\n"
    "   gl_PointSize = pointsize;\n"
    "}";

GLchar* VERTEX_SHADER =
        "#version 330 core\n"
        "layout(location = 0) in vec3 in_Vertex;\n"
        "layout(location = 1) in vec4 in_Color;\n"
        "uniform mat4 u_mvpMatrix;\n"
        "out vec4 b_color;\n"
        "void main() {\n"
        "   b_color = in_Color;\n"
        "	gl_Position = u_mvpMatrix * vec4(in_Vertex, 1);\n"
        "}";

GLchar* FRAGMENT_SHADER =
        "#version 330 core\n"
        "in vec4 b_color;\n"
        "layout(location = 0) out vec4 out_Color;\n"
        "void main() {\n"
        "   out_Color = b_color;\n"
        "}";

GLViewer* currentInstance_ = nullptr;

const float grid_size = 10.0f;

inline sl::float4 generateColorId(int idx) {
    if (idx < 0) return sl::float4(236, 184, 36, 255);
    int const offset = idx % 5;
    return  sl::float4(id_colors[offset][0], id_colors[offset][1], id_colors[offset][2], 1.f);
}

GLViewer::GLViewer() : available(false){
    currentInstance_ = this;
    mouseButton_[0] = mouseButton_[1] = mouseButton_[2] = false;
    clearInputs();
    previousMouseMotion_[0] = previousMouseMotion_[1] = 0;
}

GLViewer::~GLViewer() {}

void GLViewer::exit() {
    if (currentInstance_) 
        available = false;
}

bool GLViewer::isAvailable() {
    if(available)
        glutMainLoopEvent();
    return available;
}

void CloseFunc(void) { if(currentInstance_)  currentInstance_->exit();}

void fillZED(int nb_tri, float *vertices, int *triangles, sl::float3 color, Simple3DObject *zed_camera) {
    for (int p = 0; p < nb_tri * 3; p = p + 3) {
        int index = triangles[p] - 1;
        zed_camera->addPoint(sl::float3(vertices[index * 3], vertices[index * 3 + 1], vertices[index * 3 + 2]) * 1000, sl::float3(color.r, color.g, color.b));
        index = triangles[p + 1] - 1;
        zed_camera->addPoint(sl::float3(vertices[index * 3], vertices[index * 3 + 1], vertices[index * 3 + 2]) * 1000, sl::float3(color.r, color.g, color.b));
        index = triangles[p + 2] - 1;
        zed_camera->addPoint(sl::float3(vertices[index * 3], vertices[index * 3 + 1], vertices[index * 3 + 2]) * 1000, sl::float3(color.r, color.g, color.b));
    }
}

GLenum GLViewer::init(int argc, char **argv, 
sl::CameraParameters param, sl::FusedPointCloud* ptr, sl::MODEL zed_model) {
    bool glutInitialised = glutGet(GLUT_INIT_STATE) == 1;
    if (!glutInitialised)
    glutInit(&argc, argv);
    int wnd_w = glutGet(GLUT_SCREEN_WIDTH);
    int wnd_h = glutGet(GLUT_SCREEN_HEIGHT) *0.9;
    glutInitWindowSize(1280, 720);
    glutInitWindowPosition(wnd_w*0.05, wnd_h*0.05);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    GLint WindowID1;
    WindowID1 = glutCreateWindow("ZED PointCloud Fusion");

    GLenum err = glewInit();
    if (GLEW_OK != err)
        return err;

    glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);

    bool status_ = image_handler.initialize(param.image_size);
    if (!status_)
        std::cout << "ERROR: Failed to initialized Image Renderer" << std::endl;

    p_fpc = ptr;
    
    // Compile and create the shader
    mainShader.it = Shader(VERTEX_SHADER, FRAGMENT_SHADER);
    mainShader.MVP_Mat = glGetUniformLocation(mainShader.it.getProgramId(), "u_mvpMatrix");

    pcf_shader.it = Shader(FPC_VERTEX_SHADER, FRAGMENT_SHADER);
    pcf_shader.MVP_Mat = glGetUniformLocation(pcf_shader.it.getProgramId(), "u_mvpMatrix");

    // Create the camera
    camera_ = CameraGL(sl::Translation(0, 0, 1000), sl::Translation(0, 0, -100));
    camera_.setOffsetFromPosition(sl::Translation(0, 0, 1500));

    // change background color
    bckgrnd_clr = sl::float4(0.2f, 0.19f, 0.2f, 1.0f);


    zedPath.setDrawingType(GL_LINE_STRIP);
    zedModel.setDrawingType(GL_TRIANGLES);

    BBox_edges = Simple3DObject(sl::Translation(0, 0, 0), false);
    BBox_edges.setDrawingType(GL_LINES);

    BBox_faces = Simple3DObject(sl::Translation(0, 0, 0), false);
    BBox_faces.setDrawingType(GL_QUADS);

    skeletons = Simple3DObject(sl::Translation(0, 0, 0), false);
    skeletons.setDrawingType(GL_LINES);

    Model3D *model;
    switch (zed_model)
    {
    case sl::MODEL::ZED: model = new Model3D_ZED(); break;
    case sl::MODEL::ZED2: model = new Model3D_ZED2(); break;
    case sl::MODEL::ZED_M: model = new Model3D_ZED_M(); break;
    }
    for (auto it : model->part)
        fillZED(it.nb_triangles, model->vertices, it.triangles, it.color, &zedModel);
    delete model;

    zedModel.pushToGPU();
    updateZEDposition = false;

    // Map glut function on this class methods
    glutDisplayFunc(GLViewer::drawCallback);
    glutMouseFunc(GLViewer::mouseButtonCallback);
    glutMotionFunc(GLViewer::mouseMotionCallback);
    glutReshapeFunc(GLViewer::reshapeCallback);
    glutKeyboardFunc(GLViewer::keyPressedCallback);
    glutKeyboardUpFunc(GLViewer::keyReleasedCallback);
    glutCloseFunc(CloseFunc);

    available = true;
    
    // ready to start
    chunks_pushed = true;

    return err;
}

GLenum GLViewer::init2(int argc, char** argv,
    sl::CameraParameters param, sl::FusedPointCloud* ptr, sl::MODEL zed_model) {
    bool glutInitialised = glutGet(GLUT_INIT_STATE) == 1;
    if (!glutInitialised)
        glutInit(&argc, argv);
    int wnd_w = glutGet(GLUT_SCREEN_WIDTH);
    int wnd_h = glutGet(GLUT_SCREEN_HEIGHT) * 0.9;
    glutInitWindowSize(1280, 720);
    glutInitWindowPosition(wnd_w * 0.05, wnd_h * 0.05);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    glutCreateWindow("ZED obj det");

    GLenum err = glewInit();
    if (GLEW_OK != err)
        return err;

    glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);

    bool status_ = image_handler.initialize(param.image_size);
    if (!status_)
        std::cout << "ERROR: Failed to initialized Image Renderer" << std::endl;

    p_fpc = ptr;

    // Compile and create the shader
    mainShader.it = Shader(VERTEX_SHADER, FRAGMENT_SHADER);
    mainShader.MVP_Mat = glGetUniformLocation(mainShader.it.getProgramId(), "u_mvpMatrix");

    pcf_shader.it = Shader(FPC_VERTEX_SHADER, FRAGMENT_SHADER);
    pcf_shader.MVP_Mat = glGetUniformLocation(pcf_shader.it.getProgramId(), "u_mvpMatrix");

    // Create the camera
    camera_ = CameraGL(sl::Translation(0, 0, 1000), sl::Translation(0, 0, -100));
    camera_.setOffsetFromPosition(sl::Translation(0, 0, 1500));

    // change background color
    bckgrnd_clr = sl::float4(0.2f, 0.19f, 0.2f, 1.0f);


    zedPath.setDrawingType(GL_LINE_STRIP);
    zedModel.setDrawingType(GL_TRIANGLES);

    BBox_edges = Simple3DObject(sl::Translation(0, 0, 0), false);
    BBox_edges.setDrawingType(GL_LINES);

    BBox_faces = Simple3DObject(sl::Translation(0, 0, 0), false);
    BBox_faces.setDrawingType(GL_QUADS);

    skeletons = Simple3DObject(sl::Translation(0, 0, 0), false);
    skeletons.setDrawingType(GL_LINES);

    Model3D* model;
    switch (zed_model)
    {
    case sl::MODEL::ZED: model = new Model3D_ZED(); break;
    case sl::MODEL::ZED2: model = new Model3D_ZED2(); break;
    case sl::MODEL::ZED_M: model = new Model3D_ZED_M(); break;
    }
    for (auto it : model->part)
        fillZED(it.nb_triangles, model->vertices, it.triangles, it.color, &zedModel);
    delete model;

    zedModel.pushToGPU();
    updateZEDposition = false;

    // Map glut function on this class methods
    glutDisplayFunc(GLViewer::drawCallback);
    glutMouseFunc(GLViewer::mouseButtonCallback);
    glutMotionFunc(GLViewer::mouseMotionCallback);
    glutReshapeFunc(GLViewer::reshapeCallback);
    glutKeyboardFunc(GLViewer::keyPressedCallback);
    glutKeyboardUpFunc(GLViewer::keyReleasedCallback);
    glutCloseFunc(CloseFunc);

    available = true;

    // ready to start
    chunks_pushed = true;

    return err;
}


void GLViewer::setRenderCameraProjection(sl::CameraParameters params, float znear, float zfar) {
    // Just slightly up the ZED camera FOV to make a small black border
    float fov_y = (params.v_fov + 0.5f) * M_PI / 180.f;
    float fov_x = (params.h_fov + 0.5f) * M_PI / 180.f;

    projection_(0, 0) = 1.0f / tanf(fov_x * 0.5f);
    projection_(1, 1) = 1.0f / tanf(fov_y * 0.5f);
    projection_(2, 2) = -(zfar + znear) / (zfar - znear);
    projection_(3, 2) = -1;
    projection_(2, 3) = -(2.f * zfar * znear) / (zfar - znear);
    projection_(3, 3) = 0;

    projection_(0, 0) = 1.0f / tanf(fov_x * 0.5f); //Horizontal FoV.
    projection_(0, 1) = 0;
    projection_(0, 2) = 2.0f * ((params.image_size.width - 1.0f * params.cx) / params.image_size.width) - 1.0f; //Horizontal offset.
    projection_(0, 3) = 0;

    projection_(1, 0) = 0;
    projection_(1, 1) = 1.0f / tanf(fov_y * 0.5f); //Vertical FoV.
    projection_(1, 2) = -(2.0f * ((params.image_size.height - 1.0f * params.cy) / params.image_size.height) - 1.0f); //Vertical offset.
    projection_(1, 3) = 0;

    projection_(2, 0) = 0;
    projection_(2, 1) = 0;
    projection_(2, 2) = -(zfar + znear) / (zfar - znear); //Near and far planes.
    projection_(2, 3) = -(2.0f * zfar * znear) / (zfar - znear); //Near and far planes.

    projection_(3, 0) = 0;
    projection_(3, 1) = 0;
    projection_(3, 2) = -1;
    projection_(3, 3) = 0.0f;
}

void GLViewer::render() {
    if (available) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(bckgrnd_clr.r, bckgrnd_clr.g, bckgrnd_clr.b, 1.f);
        update();
        draw();
        printText();
        glutSwapBuffers();
        glutPostRedisplay();
    }
}

void GLViewer::update() {
    if (keyStates_['q'] == KEY_STATE::UP || keyStates_['Q'] == KEY_STATE::UP || keyStates_[27] == KEY_STATE::UP) {    
        currentInstance_->exit();
        return;
    }

    if (keyStates_['f'] == KEY_STATE::UP || keyStates_['F'] == KEY_STATE::UP) {
        followCamera = !followCamera;
        if(followCamera)
            camera_.setOffsetFromPosition(sl::Translation(0, 0, 1500));        
    }

    // Rotate camera with mouse
    if(!followCamera){
        if (mouseButton_[MOUSE_BUTTON::LEFT]) {
            camera_.rotate(sl::Rotation((float) mouseMotion_[1] * MOUSE_R_SENSITIVITY, camera_.getRight()));
            camera_.rotate(sl::Rotation((float) mouseMotion_[0] * MOUSE_R_SENSITIVITY, camera_.getVertical() * -1.f));
        }

        // Translate camera with mouse
        if (mouseButton_[MOUSE_BUTTON::RIGHT]) {
            camera_.translate(camera_.getUp() * (float) mouseMotion_[1] * MOUSE_T_SENSITIVITY);
            camera_.translate(camera_.getRight() * (float) mouseMotion_[0] * MOUSE_T_SENSITIVITY);
        }
    }

    // Zoom in with mouse wheel
    if (mouseWheelPosition_ != 0) {
        sl::Translation cur_offset = camera_.getOffsetFromPosition();
        bool zoom_ = mouseWheelPosition_ > 0;
        sl::Translation new_offset = cur_offset * (zoom_? MOUSE_UZ_SENSITIVITY : MOUSE_DZ_SENSITIVITY);
        if (zoom_) {
            if(followCamera) {
                if((new_offset.tz<500.f))
                    new_offset.tz = 500.f;                
            } else {
                if((new_offset.tz<50.f) ) 
                    new_offset.tz = 50.f;                
            }
        } else {
            if(followCamera) {
                if(new_offset.tz>5000.f)
                    new_offset.tz = 5000.f;                
            }
        }
        camera_.setOffsetFromPosition(new_offset);
    }
    
    // Update point cloud buffers    
    camera_.update();
    mtx.lock();

    BBox_edges.pushToGPU();
    BBox_faces.pushToGPU();
    skeletons.pushToGPU();

    if (updateZEDposition) {
        sl::float3 clr(0.1f, 0.5f, 0.9f);
        for(auto it: vecPath)
            zedPath.addPoint(it, clr);
        zedPath.pushToGPU();
        vecPath.clear();
        updateZEDposition = false;
    }
    
    if (new_chunks) {
        const int nb_c = p_fpc->chunks.size();
        if (nb_c > sub_maps.size()) {
            const float step = 500.f;
            size_t new_size = ((nb_c / step) + 1) * step;
            sub_maps.resize(new_size);
        }
        int c = 0;
        for (auto& it : sub_maps) {
            if ((c < nb_c) && p_fpc->chunks[c].has_been_updated)
                it.update(p_fpc->chunks[c]);
            c++;
        }

        new_chunks = false;
        chunks_pushed = true;
    }

    mtx.unlock();
    clearInputs();
}

void GLViewer::draw() {
    sl::Transform vpMatrix = camera_.getViewProjectionMatrix();

    glUseProgram(mainShader.it.getProgramId());
    glUniformMatrix4fv(mainShader.MVP_Mat, 1, GL_TRUE, vpMatrix.m);

    glLineWidth(1.f);
    zedPath.draw();

    glLineWidth(1.5f);
    BBox_edges.draw();
    glLineWidth(4.f);
    skeletons.draw();
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    BBox_faces.draw();



    glUniformMatrix4fv(mainShader.MVP_Mat, 1, GL_FALSE, (sl::Transform::transpose(zedModel.getModelMatrix()) *  sl::Transform::transpose(vpMatrix)).m);
 
    zedModel.draw();
    glUseProgram(0);

    if(sub_maps.size()){
        glPointSize(2.f);
        glUseProgram(pcf_shader.it.getProgramId());
        glUniformMatrix4fv(pcf_shader.MVP_Mat, 1, GL_TRUE, vpMatrix.m);

        for (auto &it: sub_maps)
            it.draw();
        glUseProgram(0);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }
}

void GLViewer::clearInputs() {
    mouseMotion_[0] = mouseMotion_[1] = 0;
    mouseWheelPosition_ = 0;
    for (unsigned int i = 0; i < 256; ++i)
        if (keyStates_[i] != KEY_STATE::DOWN)
            keyStates_[i] = KEY_STATE::FREE;
}

void GLViewer::drawCallback() {
    currentInstance_->render();
}

void printGL(float x, float y, const char *string) {
    glRasterPos2f(x, y);
    int len = (int) strlen(string);
    for(int i = 0; i < len; i++) {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, string[i]);
    }
}

sl::float2 compute3Dprojection(sl::float3& pt, const sl::Transform& cam, sl::Resolution wnd_size) {
    sl::float4 pt4d(pt.x, pt.y, pt.z, 1.);
    auto proj3D_cam = pt4d * cam;
    proj3D_cam.y += 1000.f;
    sl::float2 proj2D;
    proj2D.x = ((proj3D_cam.x / pt4d.w) * wnd_size.width) / (2.f * proj3D_cam.w) + wnd_size.width / 2.f;
    proj2D.y = ((proj3D_cam.y / pt4d.w) * wnd_size.height) / (2.f * proj3D_cam.w) + wnd_size.height / 2.f;
    return proj2D;
}

void GLViewer::printText() {
    if(available) {
        glColor3f(0.85f, 0.86f, 0.83f);
        printGL(-0.99f, 0.90f, "Press 'F' to un/follow the camera");

        std::string positional_tracking_state_str();
        // Show mapping state
        if ((tracking_state == sl::POSITIONAL_TRACKING_STATE::OK))
            glColor3f(0.25f, 0.99f, 0.25f);
        else
            glColor3f(0.99f, 0.25f, 0.25f);        
        std::string state_str("POSITIONAL TRACKING STATE : ");
        state_str += sl::toString(tracking_state).c_str();
        printGL(-0.99f, 0.95f, state_str.c_str());
    }

    const sl::Transform vpMatrix = camera_.getViewProjectionMatrix();
    sl::Resolution wnd_size(glutGet(GLUT_WINDOW_WIDTH), glutGet(GLUT_WINDOW_HEIGHT));
    for (auto it : objectsName) {
        auto pt2d = compute3Dprojection(it.position, vpMatrix, wnd_size);
        glColor4f(it.color.b, it.color.g, it.color.r, it.color.a);
        const auto* string = it.name.c_str();
        glWindowPos2f(pt2d.x, pt2d.y);
        int len = (int)strlen(string);
        for (int i = 0; i < len; i++)
            glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, string[i]);
    }
}

void GLViewer::mouseButtonCallback(int button, int state, int x, int y) {
    if (button < 5) {
        if (button < 3) {
            currentInstance_->mouseButton_[button] = state == GLUT_DOWN;
        } else {
            currentInstance_->mouseWheelPosition_ += button == MOUSE_BUTTON::WHEEL_UP ? 1 : -1;
        }
        currentInstance_->mouseCurrentPosition_[0] = x;
        currentInstance_->mouseCurrentPosition_[1] = y;
        currentInstance_->previousMouseMotion_[0] = x;
        currentInstance_->previousMouseMotion_[1] = y;
    }
}

void GLViewer::mouseMotionCallback(int x, int y) {
    currentInstance_->mouseMotion_[0] = x - currentInstance_->previousMouseMotion_[0];
    currentInstance_->mouseMotion_[1] = y - currentInstance_->previousMouseMotion_[1];
    currentInstance_->previousMouseMotion_[0] = x;
    currentInstance_->previousMouseMotion_[1] = y;
    glutPostRedisplay();
}

void GLViewer::reshapeCallback(int width, int height) {    
    glViewport(0, 0, width, height);
    float hfov = (180.0f / M_PI) * (2.0f * atan(width / (2.0f * 500)));
    float vfov = (180.0f / M_PI) * (2.0f * atan(height / (2.0f * 500)));
    currentInstance_->camera_.setProjection(hfov, vfov, currentInstance_->camera_.getZNear(), currentInstance_->camera_.getZFar());
}

void GLViewer::keyPressedCallback(unsigned char c, int x, int y) {
    currentInstance_->keyStates_[c] = KEY_STATE::DOWN;
    glutPostRedisplay();
}

void GLViewer::keyReleasedCallback(unsigned char c, int x, int y) {
    currentInstance_->keyStates_[c] = KEY_STATE::UP;
}

void GLViewer::idle() {
    glutPostRedisplay();
}

void GLViewer::updatePose(sl::Pose pose_, sl::POSITIONAL_TRACKING_STATE state) {
    mtx.lock();
    pose = pose_;
    tracking_state = state;
    vecPath.push_back(pose.getTranslation());
    zedModel.setRT(pose.pose_data);
    updateZEDposition = true;
    mtx.unlock();
    if(followCamera)   {
        camera_.setPosition(pose.getTranslation());
        sl::Rotation rot = pose.getRotationMatrix();
        camera_.setRotation(rot);
    }
}

Simple3DObject::Simple3DObject() : isStatic_(false) {
    vaoID_ = 0;
    drawingType_ = GL_TRIANGLES;
    position_ = sl::float3(0, 0, 0);
    rotation_.setIdentity();
}

Simple3DObject::Simple3DObject(sl::Translation position, bool isStatic) : isStatic_(isStatic) {
    vaoID_ = 0;
    drawingType_ = GL_TRIANGLES;
    position_ = position;
    rotation_.setIdentity();
}

Simple3DObject::~Simple3DObject() {
    if (vaoID_ != 0) {
        glDeleteBuffers(3, vboID_);
        glDeleteVertexArrays(1, &vaoID_);
    }
}

void Simple3DObject::addPoint(sl::float3 position, sl::float3 color) {
    addPoint(position.x, position.y, position.z, color.r, color.g, color.b);
}

void Simple3DObject::addPoint(float x, float y, float z, float r, float g, float b) {
    vertices_.push_back(x);
    vertices_.push_back(y);
    vertices_.push_back(z);
    colors_.push_back(r);
    colors_.push_back(g);
    colors_.push_back(b);
    indices_.push_back((int)indices_.size());
}

void Simple3DObject::addPoint(float x, float y, float z, float r, float g, float b, float a) {
    vertices_.push_back(x);
    vertices_.push_back(y);
    vertices_.push_back(z);
    colors_.push_back(r);
    colors_.push_back(g);
    colors_.push_back(b);
    colors_.push_back(a);
    indices_.push_back((int)indices_.size());
}

void Simple3DObject::addPt(sl::float3 pt) {
    vertices_.push_back(pt.x);
    vertices_.push_back(pt.y);
    vertices_.push_back(pt.z);
}

void Simple3DObject::addClr(sl::float4 clr) {
    colors_.push_back(clr.b);
    colors_.push_back(clr.g);
    colors_.push_back(clr.r);
    colors_.push_back(clr.a);
}

void Simple3DObject::addLine(sl::float3 p1, sl::float3 p2, sl::float3 clr) {
    vertices_.push_back(p1.x);
    vertices_.push_back(p1.y);
    vertices_.push_back(p1.z);

    vertices_.push_back(p2.x);
    vertices_.push_back(p2.y);
    vertices_.push_back(p2.z);

    colors_.push_back(clr.r);
    colors_.push_back(clr.g);
    colors_.push_back(clr.b);

    colors_.push_back(clr.r);
    colors_.push_back(clr.g);
    colors_.push_back(clr.b);

    indices_.push_back((int)indices_.size());
    indices_.push_back((int)indices_.size());
}

void Simple3DObject::pushToGPU() {
    if (!isStatic_ || vaoID_ == 0) {
        if (vaoID_ == 0) {
            glGenVertexArrays(1, &vaoID_);
            glGenBuffers(3, vboID_);
        }

        if (vertices_.size() > 0) {
            glBindVertexArray(vaoID_);
            glBindBuffer(GL_ARRAY_BUFFER, vboID_[0]);
            glBufferData(GL_ARRAY_BUFFER, vertices_.size() * sizeof(float), &vertices_[0], isStatic_ ? GL_STATIC_DRAW : GL_DYNAMIC_DRAW);
            glVertexAttribPointer(Shader::ATTRIB_VERTICES_POS, 3, GL_FLOAT, GL_FALSE, 0, 0);
            glEnableVertexAttribArray(Shader::ATTRIB_VERTICES_POS);
        }

        if (colors_.size() > 0) {
            glBindBuffer(GL_ARRAY_BUFFER, vboID_[1]);
            glBufferData(GL_ARRAY_BUFFER, colors_.size() * sizeof(float), &colors_[0], isStatic_ ? GL_STATIC_DRAW : GL_DYNAMIC_DRAW);
            glVertexAttribPointer(Shader::ATTRIB_COLOR_POS, 4, GL_FLOAT, GL_FALSE, 0, 0);
            glEnableVertexAttribArray(Shader::ATTRIB_COLOR_POS);
        }

        if (indices_.size() > 0) {
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vboID_[2]);
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices_.size() * sizeof(unsigned int), &indices_[0], isStatic_ ? GL_STATIC_DRAW : GL_DYNAMIC_DRAW);
        }

        glBindVertexArray(0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }
}

void Simple3DObject::clear() {
    vertices_.clear();
    colors_.clear();
    indices_.clear();
}

void GLViewer::updateView(sl::Mat image, sl::Objects& objs)
{
    mtx.lock();
    // Update Image
    image_handler.pushNewImage(image);

    // Clear frames object
    BBox_edges.clear();
    BBox_faces.clear();
    objectsName.clear();

    for (unsigned int i = 0; i < objs.object_list.size(); i++) {
        if (renderObject(objs.object_list[i])) {
            auto bb_ = objs.object_list[i].bounding_box;
            if (!bb_.empty()) {
                auto clr_class = getColorClass((int)objs.object_list[i].label);
                auto clr_id = generateColorId(objs.object_list[i].id);

                if (objs.object_list[i].tracking_state != sl::OBJECT_TRACKING_STATE::OK)
                    clr_id = clr_class;
                else
                {
                    sl::float3 pos(objs.object_list[i].position.x, objs.object_list[i].bounding_box[0].y, objs.object_list[i].position.z);
                    createIDRendering(pos, clr_id, objs.object_list[i].id);
                }
                createBboxRendering(bb_, clr_id);
            }
        }
    }
    mtx.unlock();
}

void GLViewer::updateData(std::vector<sl::ObjectData>& objs, sl::Transform& pose) {
    mtx.lock();
    BBox_edges.clear();
    BBox_faces.clear();
    objectsName.clear();
    skeletons.clear();
    cam_pose = pose;
    sl::float3 tr_0(0, 0, 0);
    cam_pose.setTranslation(tr_0);

    for (unsigned int i = 0; i < objs.size(); i++) {
        if (renderObject(objs[i])) {
            auto bb_ = objs[i].bounding_box;
            if (!bb_.empty()) {
                auto clr_class = getColorClass((int)objs[i].label);
                auto clr_id = generateColorID_f(objs[i].id);

                if (objs[i].tracking_state != sl::OBJECT_TRACKING_STATE::OK)
                    clr_id = clr_class;
                else
                    createIDRendering(objs[i].position, clr_id, objs[i].id);

                if (0) { // display centroid as a cross
                    sl::float3 centroid = objs[i].position;
                    const float size_cendtroid = 50; // mm
                    sl::float3 centroid1 = centroid;
                    centroid1.y += size_cendtroid;
                    sl::float3 centroid2 = objs[i].position;
                    centroid2.y -= size_cendtroid;
                    sl::float3 centroid3 = objs[i].position;
                    centroid3.x += size_cendtroid;
                    sl::float3 centroid4 = objs[i].position;
                    centroid4.x -= size_cendtroid;

                    BBox_edges.addLine(centroid1, centroid2, sl::float4(1.f, 0.5f, 0.5f, 1.f));
                    BBox_edges.addLine(centroid3, centroid4, sl::float4(1.f, 0.5f, 0.5f, 1.f));
                }

                //Display sekeleton if available
                auto clr_bones = generateColorID_f(objs[i].id);
                auto keypoints = objs[i].keypoint;
                if (keypoints.size() > 0) {
                    for (auto& limb : sl::BODY_BONES) {
                        sl::float3 kp_1 = keypoints[(int)limb.first];
                        sl::float3 kp_2 = keypoints[(int)limb.second];
                        if (std::isfinite(kp_1.x) && std::isfinite(kp_2.x))
                            skeletons.addLine(kp_1, kp_2, clr_bones);
                    }
                }
                createBboxRendering(bb_, clr_id);
            }
        }
    }
    mtx.unlock();
}

void Simple3DObject::setDrawingType(GLenum type) {
    drawingType_ = type;
}

void Simple3DObject::draw() {
    if (indices_.size() && vaoID_) {
        glBindVertexArray(vaoID_);
        glDrawElements(drawingType_, (GLsizei)indices_.size(), GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
    }
}

void Simple3DObject::translate(const sl::Translation& t) {
    position_ = position_ + t;
}

void Simple3DObject::setPosition(const sl::Translation& p) {
    position_ = p;
}

void Simple3DObject::setRT(const sl::Transform& mRT) {
    position_ = mRT.getTranslation();
    rotation_ = mRT.getOrientation();
}

void Simple3DObject::rotate(const sl::Orientation& rot) {
    rotation_ = rot * rotation_;
}

void Simple3DObject::rotate(const sl::Rotation& m) {
    this->rotate(sl::Orientation(m));
}

void Simple3DObject::setRotation(const sl::Orientation& rot) {
    rotation_ = rot;
}

void Simple3DObject::setRotation(const sl::Rotation& m) {
    this->setRotation(sl::Orientation(m));
}

const sl::Translation& Simple3DObject::getPosition() const {
    return position_;
}

sl::Transform Simple3DObject::getModelMatrix() const {
    sl::Transform tmp = sl::Transform::identity();
    tmp.setOrientation(rotation_);
    tmp.setTranslation(position_);
    return tmp;
}

Shader::Shader(GLchar* vs, GLchar* fs) {
    if (!compile(verterxId_, GL_VERTEX_SHADER, vs)) {
        std::cout << "ERROR: while compiling vertex shader" << std::endl;
    }
    if (!compile(fragmentId_, GL_FRAGMENT_SHADER, fs)) {
        std::cout << "ERROR: while compiling fragment shader" << std::endl;
    }

    programId_ = glCreateProgram();

    glAttachShader(programId_, verterxId_);
    glAttachShader(programId_, fragmentId_);

    glBindAttribLocation(programId_, ATTRIB_VERTICES_POS, "in_vertex");
    glBindAttribLocation(programId_, ATTRIB_COLOR_POS, "in_texCoord");

    glLinkProgram(programId_);

    GLint errorlk(0);
    glGetProgramiv(programId_, GL_LINK_STATUS, &errorlk);
    if (errorlk != GL_TRUE) {
        std::cout << "ERROR: while linking Shader :" << std::endl;
        GLint errorSize(0);
        glGetProgramiv(programId_, GL_INFO_LOG_LENGTH, &errorSize);

        char *error = new char[errorSize + 1];
        glGetShaderInfoLog(programId_, errorSize, &errorSize, error);
        error[errorSize] = '\0';
        std::cout << error << std::endl;

        delete[] error;
        glDeleteProgram(programId_);
    }
}

Shader::~Shader() {
    if (verterxId_ != 0)
        glDeleteShader(verterxId_);
    if (fragmentId_ != 0)
        glDeleteShader(fragmentId_);
    if (programId_ != 0)
        glDeleteShader(programId_);
}

GLuint Shader::getProgramId() {
    return programId_;
}

bool Shader::compile(GLuint &shaderId, GLenum type, GLchar* src) {
    shaderId = glCreateShader(type);
    if (shaderId == 0) {
        std::cout << "ERROR: shader type (" << type << ") does not exist" << std::endl;
        return false;
    }
    glShaderSource(shaderId, 1, (const char**)&src, 0);
    glCompileShader(shaderId);

    GLint errorCp(0);
    glGetShaderiv(shaderId, GL_COMPILE_STATUS, &errorCp);
    if (errorCp != GL_TRUE) {
        std::cout << "ERROR: while compiling Shader :" << std::endl;
        GLint errorSize(0);
        glGetShaderiv(shaderId, GL_INFO_LOG_LENGTH, &errorSize);

        char *error = new char[errorSize + 1];
        glGetShaderInfoLog(shaderId, errorSize, &errorSize, error);
        error[errorSize] = '\0';
        std::cout << error << std::endl;

        delete[] error;
        glDeleteShader(shaderId);
        return false;
    }
    return true;
}

const sl::Translation CameraGL::ORIGINAL_FORWARD = sl::Translation(0, 0, 1);
const sl::Translation CameraGL::ORIGINAL_UP = sl::Translation(0, 1, 0);
const sl::Translation CameraGL::ORIGINAL_RIGHT = sl::Translation(1, 0, 0);

CameraGL::CameraGL(sl::Translation position, sl::Translation direction, sl::Translation vertical) {
    this->position_ = position;
    setDirection(direction, vertical);

    offset_ = sl::Translation(0, 0, 0);
    view_.setIdentity();
    updateView();
    setProjection(80, 80, 100.f, 900000.f);
    updateVPMatrix();
}

CameraGL::~CameraGL() {}

void CameraGL::update() {
    if (sl::Translation::dot(vertical_, up_) < 0)
        vertical_ = vertical_ * -1.f;
    updateView();
    updateVPMatrix();
}

void GLViewer::createIDRendering(sl::float3& center, sl::float4 clr, unsigned int id) {
    ObjectClassName tmp;
    tmp.name = "ID: " + std::to_string(id);
    tmp.color = clr;
    tmp.position = center; // Reference point
    objectsName.push_back(tmp);
}

void Simple3DObject::addTopFace(std::vector<sl::float3>& pts, sl::float4 clr) {
    clr.a = 0.3f;
    for (auto it : pts) {
        addPt(it);
        addClr(clr);
        indices_.push_back((int)indices_.size());
    }
}

void Simple3DObject::addVerticalFaces(std::vector<sl::float3>& pts, sl::float4 clr) {
    auto addQuad = [&](std::vector<sl::float3> quad_pts, float alpha1, float alpha2) { // To use only with 4 points
        for (unsigned int i = 0; i < quad_pts.size(); ++i) {
            addPt(quad_pts[i]);
            clr.a = (i < 2 ? alpha1 : alpha2);
            addClr(clr);
        }

        indices_.push_back((int)indices_.size());
        indices_.push_back((int)indices_.size());
        indices_.push_back((int)indices_.size());
        indices_.push_back((int)indices_.size());
    };

    // For each face, we need to add 4 quads (the first 2 indexes are always the top points of the quad)
    std::vector<std::vector<int>> quads
    {
        {
            0, 3, 7, 4
        }, // front face
        {
            3, 2, 6, 7
        }, // right face
        {
            2, 1, 5, 6
        }, // back face
        {
            1, 0, 4, 5
        } // left face
    };
    float alpha = 0.5f;

    for (const auto quad : quads) {

        // Top quads
        std::vector<sl::float3> quad_pts_1{
            pts[quad[0]],
            pts[quad[1]],
            ((grid_size - 0.5f) * pts[quad[1]] + 0.5f * pts[quad[2]]) / grid_size,
            ((grid_size - 0.5f) * pts[quad[0]] + 0.5f * pts[quad[3]]) / grid_size };
        addQuad(quad_pts_1, alpha, alpha);

        std::vector<sl::float3> quad_pts_2{
            ((grid_size - 0.5f) * pts[quad[0]] + 0.5f * pts[quad[3]]) / grid_size,
            ((grid_size - 0.5f) * pts[quad[1]] + 0.5f * pts[quad[2]]) / grid_size,
            ((grid_size - 1.0f) * pts[quad[1]] + pts[quad[2]]) / grid_size,
            ((grid_size - 1.0f) * pts[quad[0]] + pts[quad[3]]) / grid_size };
        addQuad(quad_pts_2, alpha, 2 * alpha / 3);

        std::vector<sl::float3> quad_pts_3{
            ((grid_size - 1.0f) * pts[quad[0]] + pts[quad[3]]) / grid_size,
            ((grid_size - 1.0f) * pts[quad[1]] + pts[quad[2]]) / grid_size,
            ((grid_size - 1.5f) * pts[quad[1]] + 1.5f * pts[quad[2]]) / grid_size,
            ((grid_size - 1.5f) * pts[quad[0]] + 1.5f * pts[quad[3]]) / grid_size };
        addQuad(quad_pts_3, 2 * alpha / 3, alpha / 3);

        std::vector<sl::float3> quad_pts_4{
            ((grid_size - 1.5f) * pts[quad[0]] + 1.5f * pts[quad[3]]) / grid_size,
            ((grid_size - 1.5f) * pts[quad[1]] + 1.5f * pts[quad[2]]) / grid_size,
            ((grid_size - 2.0f) * pts[quad[1]] + 2.0f * pts[quad[2]]) / grid_size,
            ((grid_size - 2.0f) * pts[quad[0]] + 2.0f * pts[quad[3]]) / grid_size };
        addQuad(quad_pts_4, alpha / 3, 0.0f);

        // Bottom quads
        std::vector<sl::float3> quad_pts_5{
            (pts[quad[1]] * 2.0f + (grid_size - 2.0f) * pts[quad[2]]) / grid_size,
            (pts[quad[0]] * 2.0f + (grid_size - 2.0f) * pts[quad[3]]) / grid_size,
            (pts[quad[0]] * 1.5f + (grid_size - 1.5f) * pts[quad[3]]) / grid_size,
            (pts[quad[1]] * 1.5f + (grid_size - 1.5f) * pts[quad[2]]) / grid_size };
        addQuad(quad_pts_5, 0.0f, alpha / 3);

        std::vector<sl::float3> quad_pts_6{
            (pts[quad[1]] * 1.5f + (grid_size - 1.5f) * pts[quad[2]]) / grid_size,
            (pts[quad[0]] * 1.5f + (grid_size - 1.5f) * pts[quad[3]]) / grid_size,
            (pts[quad[0]] + (grid_size - 1.0f) * pts[quad[3]]) / grid_size,
            (pts[quad[1]] + (grid_size - 1.0f) * pts[quad[2]]) / grid_size };
        addQuad(quad_pts_6, alpha / 3, 2 * alpha / 3);

        std::vector<sl::float3> quad_pts_7{
            (pts[quad[1]] + (grid_size - 1.0f) * pts[quad[2]]) / grid_size,
            (pts[quad[0]] + (grid_size - 1.0f) * pts[quad[3]]) / grid_size,
            (pts[quad[0]] * 0.5f + (grid_size - 0.5f) * pts[quad[3]]) / grid_size,
            (pts[quad[1]] * 0.5f + (grid_size - 0.5f) * pts[quad[2]]) / grid_size };
        addQuad(quad_pts_7, 2 * alpha / 3, alpha);

        std::vector<sl::float3> quad_pts_8{
            (pts[quad[0]] * 0.5f + (grid_size - 0.5f) * pts[quad[3]]) / grid_size,
            (pts[quad[1]] * 0.5f + (grid_size - 0.5f) * pts[quad[2]]) / grid_size,
            pts[quad[2]],
            pts[quad[3]] };
        addQuad(quad_pts_8, alpha, alpha);
    }
}

void Simple3DObject::addFullEdges(std::vector<sl::float3>& pts, sl::float4 clr) {
    clr.w = 0.4f;

    int start_id = vertices_.size() / 3;

    for (unsigned int i = 0; i < pts.size(); i++) {
        addPt(pts[i]);
        addClr(clr);
    }

    const std::vector<int> boxLinksTop = { 0, 1, 1, 2, 2, 3, 3, 0 };
    for (unsigned int i = 0; i < boxLinksTop.size(); i += 2) {
        indices_.push_back(start_id + boxLinksTop[i]);
        indices_.push_back(start_id + boxLinksTop[i + 1]);
    }

    const std::vector<int> boxLinksBottom = { 4, 5, 5, 6, 6, 7, 7, 4 };
    for (unsigned int i = 0; i < boxLinksBottom.size(); i += 2) {
        indices_.push_back(start_id + boxLinksBottom[i]);
        indices_.push_back(start_id + boxLinksBottom[i + 1]);
    }
}

void Simple3DObject::addVerticalEdges(std::vector<sl::float3>& pts, sl::float4 clr) {
    auto addSingleVerticalLine = [&](sl::float3 top_pt, sl::float3 bot_pt) {
        std::vector<sl::float3> current_pts{
            top_pt,
                    ((grid_size - 1.0f) * top_pt + bot_pt) / grid_size,
                    ((grid_size - 2.0f) * top_pt + bot_pt * 2.0f) / grid_size,
                    (2.0f * top_pt + bot_pt * (grid_size - 2.0f)) / grid_size,
                    (top_pt + bot_pt * (grid_size - 1.0f)) / grid_size,
                    bot_pt };

        int start_id = vertices_.size() / 3;

        for (unsigned int i = 0; i < current_pts.size(); i++) {
            addPt(current_pts[i]);
            clr.a = (i == 2 || i == 3) ? 0.0f : 0.4f;
            addClr(clr);
        }

        const std::vector<int> boxLinks = { 0, 1, 1, 2, 2, 3, 3, 4, 4, 5 };
        for (unsigned int i = 0; i < boxLinks.size(); i += 2) {
            indices_.push_back(start_id + boxLinks[i]);
            indices_.push_back(start_id + boxLinks[i + 1]);
        }
    };

    addSingleVerticalLine(pts[0], pts[4]);
    addSingleVerticalLine(pts[1], pts[5]);
    addSingleVerticalLine(pts[2], pts[6]);
    addSingleVerticalLine(pts[3], pts[7]);
}

void GLViewer::createBboxRendering(std::vector<sl::float3>& bbox, sl::float4 bbox_clr) {
    // First create top and bottom full edges
    BBox_edges.addFullEdges(bbox, bbox_clr);
    // Add faded vertical edges
    BBox_edges.addVerticalEdges(bbox, bbox_clr);
    // Add faces
    BBox_faces.addVerticalFaces(bbox, bbox_clr);
    // Add top face
    BBox_faces.addTopFace(bbox, bbox_clr);
}

void CameraGL::setProjection(float horizontalFOV, float verticalFOV, float znear, float zfar) {
    horizontalFieldOfView_ = horizontalFOV;
    verticalFieldOfView_ = verticalFOV;
    znear_ = znear;
    zfar_ = zfar;

    float fov_y = verticalFOV * M_PI / 180.f;
    float fov_x = horizontalFOV * M_PI / 180.f;

    projection_.setIdentity();
    projection_(0, 0) = 1.0f / tanf(fov_x * 0.5f);
    projection_(1, 1) = 1.0f / tanf(fov_y * 0.5f);
    projection_(2, 2) = -(zfar + znear) / (zfar - znear);
    projection_(3, 2) = -1;
    projection_(2, 3) = -(2.f * zfar * znear) / (zfar - znear);
    projection_(3, 3) = 0;
}

const sl::Transform& CameraGL::getViewProjectionMatrix() const {
    return vpMatrix_;
}

float CameraGL::getHorizontalFOV() const {
    return horizontalFieldOfView_;
}

float CameraGL::getVerticalFOV() const {
    return verticalFieldOfView_;
}

void CameraGL::setOffsetFromPosition(const sl::Translation& o) {
    offset_ = o;
}

const sl::Translation& CameraGL::getOffsetFromPosition() const {
    return offset_;
}

void CameraGL::setDirection(const sl::Translation& direction, const sl::Translation& vertical) {
    sl::Translation dirNormalized = direction;
    dirNormalized.normalize();
    this->rotation_ = sl::Orientation(ORIGINAL_FORWARD, dirNormalized * -1.f);
    updateVectors();
    this->vertical_ = vertical;
    if (sl::Translation::dot(vertical_, up_) < 0)
        rotate(sl::Rotation(M_PI, ORIGINAL_FORWARD));
}

void CameraGL::translate(const sl::Translation& t) {
    position_ = position_ + t;
}

void CameraGL::setPosition(const sl::Translation& p) {
    position_ = p;
}

void CameraGL::rotate(const sl::Orientation& rot) {
    rotation_ = rot * rotation_;
    updateVectors();
}

void CameraGL::rotate(const sl::Rotation& m) {
    this->rotate(sl::Orientation(m));
}

void CameraGL::setRotation(const sl::Orientation& rot) {
    rotation_ = rot;
    updateVectors();
}

void CameraGL::setRotation(const sl::Rotation& m) {
    this->setRotation(sl::Orientation(m));
}

const sl::Translation& CameraGL::getPosition() const {
    return position_;
}

const sl::Translation& CameraGL::getForward() const {
    return forward_;
}

const sl::Translation& CameraGL::getRight() const {
    return right_;
}

const sl::Translation& CameraGL::getUp() const {
    return up_;
}

const sl::Translation& CameraGL::getVertical() const {
    return vertical_;
}

float CameraGL::getZNear() const {
    return znear_;
}

float CameraGL::getZFar() const {
    return zfar_;
}

void CameraGL::updateVectors() {
    forward_ = ORIGINAL_FORWARD * rotation_;
    up_ = ORIGINAL_UP * rotation_;
    right_ = sl::Translation(ORIGINAL_RIGHT * -1.f) * rotation_;
}

void CameraGL::updateView() {
    sl::Transform transformation(rotation_, (offset_ * rotation_) + position_);
    view_ = sl::Transform::inverse(transformation);
}

void CameraGL::updateVPMatrix() {
    vpMatrix_ = projection_ * view_;
}

SubMapObj::SubMapObj() {
    current_fc = 0;
    vaoID_ = 0;
}

SubMapObj::~SubMapObj() {
    current_fc = 0;
    if(vaoID_) {
        glDeleteBuffers(2, vboID_);
        glDeleteVertexArrays(1, &vaoID_);
    }
}

void SubMapObj::update(sl::PointCloudChunk &chunk) {
    if (vaoID_ == 0) {
        glGenVertexArrays(1, &vaoID_);
        glGenBuffers(2, vboID_);
    }

    glShadeModel(GL_SMOOTH);

	const auto nb_v = chunk.vertices.size();
    index.resize(nb_v);
    for (int c = 0; c < nb_v; c++) index[c] = c;
    
    glBindVertexArray(vaoID_);

    glBindBuffer(GL_ARRAY_BUFFER, vboID_[Shader::ATTRIB_VERTICES_POS]);
    glBufferData(GL_ARRAY_BUFFER, chunk.vertices.size() * sizeof(sl::float4), &chunk.vertices[0], GL_DYNAMIC_DRAW);
    glVertexAttribPointer(Shader::ATTRIB_VERTICES_POS, 4, GL_FLOAT, GL_FALSE, 0, 0);
    glEnableVertexAttribArray(Shader::ATTRIB_VERTICES_POS);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vboID_[1]);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, index.size() * sizeof(sl::uint1), &index[0], GL_DYNAMIC_DRAW);
    current_fc = (int)index.size();
    
    glBindVertexArray(0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void SubMapObj::draw() {
    if(current_fc && vaoID_) {
        glBindVertexArray(vaoID_);
        glDrawElements(GL_POINTS, (GLsizei) current_fc, GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
    }
}

GLchar* IMAGE_FRAGMENT_SHADER =
"#version 330 core\n"
" in vec2 UV;\n"
" out vec4 color;\n"
" uniform sampler2D texImage;\n"
" uniform bool revert;\n"
" uniform bool rgbflip;\n"
" void main() {\n"
"    vec2 scaler  =revert?vec2(UV.x,1.f - UV.y):vec2(UV.x,UV.y);\n"
"    vec3 rgbcolor = rgbflip?vec3(texture(texImage, scaler).zyx):vec3(texture(texImage, scaler).xyz);\n"
" float gamma = 1.0/1.65;\n"
"   vec3 color_rgb = pow(rgbcolor, vec3(1.0/gamma));;\n"
"    color = vec4(color_rgb,1);\n"
"}";

GLchar* IMAGE_VERTEX_SHADER =
"#version 330\n"
"layout(location = 0) in vec3 vert;\n"
"out vec2 UV;"
"void main() {\n"
"   UV = (vert.xy+vec2(1,1))/2;\n"
"	gl_Position = vec4(vert, 1);\n"
"}\n";

ImageHandler::ImageHandler() {}

ImageHandler::~ImageHandler() {
    close();
}

void ImageHandler::close() {
    glDeleteTextures(1, &imageTex);
}

bool ImageHandler::initialize(sl::Resolution res) {
    shaderImage.it = Shader(IMAGE_VERTEX_SHADER, IMAGE_FRAGMENT_SHADER);
    texID = glGetUniformLocation(shaderImage.it.getProgramId(), "texImage");
    static const GLfloat g_quad_vertex_buffer_data[] = {
        -1.0f, -1.0f, 0.0f,
        1.0f, -1.0f, 0.0f,
        -1.0f, 1.0f, 0.0f,
        -1.0f, 1.0f, 0.0f,
        1.0f, -1.0f, 0.0f,
        1.0f, 1.0f, 0.0f };

    glGenBuffers(1, &quad_vb);
    glBindBuffer(GL_ARRAY_BUFFER, quad_vb);
    glBufferData(GL_ARRAY_BUFFER, sizeof(g_quad_vertex_buffer_data), g_quad_vertex_buffer_data, GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glEnable(GL_TEXTURE_2D);
    glGenTextures(1, &imageTex);
    glBindTexture(GL_TEXTURE_2D, imageTex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, res.width, res.height, 0, GL_BGRA_EXT, GL_UNSIGNED_BYTE, NULL);
    glBindTexture(GL_TEXTURE_2D, 0);
    cudaError_t err = cudaGraphicsGLRegisterImage(&cuda_gl_ressource, imageTex, GL_TEXTURE_2D, cudaGraphicsRegisterFlagsWriteDiscard);
    return (err == cudaSuccess);
}

void ImageHandler::pushNewImage(sl::Mat& image) {
    cudaArray_t ArrIm;
    cudaGraphicsMapResources(1, &cuda_gl_ressource, 0);
    cudaGraphicsSubResourceGetMappedArray(&ArrIm, cuda_gl_ressource, 0, 0);
    cudaMemcpy2DToArray(ArrIm, 0, 0, image.getPtr<sl::uchar1>(sl::MEM::GPU), image.getStepBytes(sl::MEM::GPU), image.getPixelBytes() * image.getWidth(), image.getHeight(), cudaMemcpyDeviceToDevice);
    cudaGraphicsUnmapResources(1, &cuda_gl_ressource, 0);
}

void ImageHandler::draw() {
    glUseProgram(shaderImage.it.getProgramId());
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, imageTex);
    glUniform1i(texID, 0);
    //invert y axis and color for this image (since its reverted from cuda array)
    glUniform1i(glGetUniformLocation(shaderImage.it.getProgramId(), "revert"), 1);
    glUniform1i(glGetUniformLocation(shaderImage.it.getProgramId(), "rgbflip"), 1);

    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, quad_vb);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glDisableVertexAttribArray(0);
    glUseProgram(0);
}