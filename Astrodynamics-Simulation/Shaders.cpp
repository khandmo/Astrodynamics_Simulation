#include "Shaders.h"

// definition of function that returns file contents
// as std string
std::string get_file_contents(const char* filename) {
	std::ifstream in(filename, std::ios::binary);
	if (in) {
		std::string contents;
		in.seekg(0, std::ios::end);
		contents.resize(in.tellg());
		in.seekg(0, std::ios::beg);
		in.read(&contents[0], contents.size());
		in.close();
		return(contents);
	}
	throw(errno);
}

// all below code snatched from main for organization
// names changed appropriately, self references changed to 
// "ID"
Shader::Shader(const char* vertexFile, const char* fragmentFile) {
	std::string vertexCode = get_file_contents(vertexFile);
	std::string fragmentCode = get_file_contents(fragmentFile);
	int success;
	char infoLog[512];

	// string to char array
	const char* vertexSource = vertexCode.c_str();
	const char* fragmentSource = fragmentCode.c_str();

	//Creating vertex shaders - position processing
	GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
	//Connects shader source code to shader object just created
	glShaderSource(vertexShader, 1, &vertexSource, NULL);
	//Compile vertex shader to machine code to execute @ runtime
	glCompileShader(vertexShader);
	//Check for errors
	glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
	if (!success) {
		glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
		std::cout << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n" << infoLog << std::endl;
	};

	//Creating fragment shaders - for color output processing
	GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
	//Connects fragment source code to fragment object just created
	glShaderSource(fragmentShader, 1, &fragmentSource, NULL);
	//Compile fragment shader to machine code to execute @runtime
	glCompileShader(fragmentShader);
	//Check for errors
	glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
	if (!success) {
		glGetShaderInfoLog(fragmentShader, 512, NULL, infoLog);
		std::cout << "ERROR::SHADER::FRAGMENT:COMPILATION_FAILED\n" << infoLog << std::endl;
	};


	//Wrap shaders in shader program - controls changing from
	//one shader/frag batch to another

	//Create shader program
	ID = glCreateProgram();
	//attach vertex and fragment shaders to the program
	glAttachShader(ID, vertexShader);
	glAttachShader(ID, fragmentShader);
	//links output of shader to input of next shader
	glLinkProgram(ID);
	//Check for errors
	glGetProgramiv(ID, GL_LINK_STATUS, &success);
	if (!success) {
		glGetProgramInfoLog(ID, 512, NULL, infoLog);
		std::cout << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n" << infoLog << std::endl;
	}

	//Delete shaders (now wrapped in shader program)
	glDeleteShader(vertexShader);
	glDeleteShader(fragmentShader);
}

void Shader::Activate() {
	// packaged shader program is referenced in ID and this executes it
	glUseProgram(ID);
}

void Shader::Delete() {
	// shader program reference in ID is deleted
	glDeleteProgram(ID);
}