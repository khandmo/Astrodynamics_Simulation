#include "Marker.h"

glm::vec2 simToScreen(glm::vec3 simPos, glm::mat4 view, glm::mat4 proj, int screenWidth, int screenHeight);
 
void checkShaderCompile(GLuint shader) {
    GLint success;
    GLchar infoLog[512];
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(shader, 512, NULL, infoLog);
        std::cerr << "Shader Compilation Failed: " << infoLog << std::endl;
    }
}

Marker::Marker(int type, float size, glm::vec3 fixedPos, glm::vec3 corr) {
    // establish types in struct that hold necessary info
    // initialize shaders

    marker_type = type;

    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    std::string sVert = get_file_contents("simple.vert");
    std::string sFrag = get_file_contents("simple.frag");
    const char* vert = sVert.c_str();
    const char* frag = sFrag.c_str();

    GLenum err = glGetError();
    if (err != GL_NO_ERROR) {
        printf("GL Error before shader creation: %d\n", err);
    }
    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vert, NULL);
    glCompileShader(vertexShader);
    checkShaderCompile(vertexShader);

    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &frag, NULL);
    glCompileShader(fragmentShader);
    checkShaderCompile(fragmentShader);

    shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);
    err = glGetError();
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    Marker::fixedPos = fixedPos;
    Marker::corrPos = corr;


}

void Marker::MarkerRender(Camera* camera) {
    float aspectRatio = camera->width / (float)camera->height;

    Marker::screenPos = simToScreen(fixedPos + corrPos, camera->view, camera->proj, camera->width, camera->height);

    float ndcTriangleSize = 0.03f / aspectRatio; // adjust based on screen size

    // define triangle directly in NDC space
    glm::vec2 ndcP1 = screenPos + glm::vec2(0, (ndcTriangleSize - 0.03f) / 2);      // Top
    glm::vec2 ndcP2 = screenPos + glm::vec2(-ndcTriangleSize / 2, ndcTriangleSize * 3 / 5); // Left
    glm::vec2 ndcP3 = screenPos + glm::vec2(ndcTriangleSize / 2, ndcTriangleSize * 3/ 5);  // Right



    if (marker_type == 1) { // capsule marker
        ndcP1 = screenPos + glm::vec2(0, (ndcTriangleSize * sqrt(3) / 3) + ((0.03f * sqrt(3) / 3) - (ndcTriangleSize * sqrt(3) / 3)));  // Top
        ndcP2 = screenPos + glm::vec2(-ndcTriangleSize / 2, -ndcTriangleSize * sqrt(3) / 6); // Left
        ndcP3 = screenPos + glm::vec2(ndcTriangleSize / 2, -ndcTriangleSize * sqrt(3) / 6);  // Right

        color = glm::vec4(0.24f, 0.28f, 0.45f, 1.0f);
    }

    float vertices[] = {
    ndcP1.x, ndcP1.y,
    color.x, color.y,
    color.z, color.w,

    ndcP2.x, ndcP2.y,
    color.x, color.y,
    color.z, color.w,

    ndcP3.x, ndcP3.y,
    color.x, color.y,
    color.z, color.w
    };


    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), &vertices, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(2 * sizeof(float)));
    
    
    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);

    glUseProgram(shaderProgram);
    glBindVertexArray(VAO);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glBindVertexArray(0);


}

Marker::~Marker() {
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteProgram(shaderProgram);
}

glm::vec2 simToScreen(glm::vec3 simPos, glm::mat4 view, glm::mat4 proj, int screenWidth, int screenHeight) {
    glm::vec4 clipSpace = proj * view * glm::vec4(simPos, 1.0f);

    // Perspective divide
    clipSpace /= clipSpace.w;

    return clipSpace;
}

