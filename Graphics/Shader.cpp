
#include "Shader.h"
#include <fstream>
#include <iostream>

Shader::Shader()
{
	numShaders = 0;
	shaders[VERTEX_SHADER] = 0;
	shaders[FRAGMENT_SHADER] = 0;
	shaders[GEOMETRY_SHADER] = 0;
	attributes.clear();
	uniforms.clear();
}

void Shader::LoadFromFile(const GLenum type, const std::string& fileName)
{
	std::ifstream fp;
	fp.open(fileName.c_str(), std::ios_base::in);
	if (fp)
	{
		std::string line, src;
		while (getline(fp, line))
		{
			src.append(line);
			src.append("\n");
		}

		CompileShader(type, src);
	}
	else
	{
		std::cerr << "Error loading shader: " << fileName << std::endl;
	}
}

void Shader::CompileShader(const GLenum type, const std::string& source)
{
	GLuint shaderID = glCreateShader(type);

	const char* src = source.c_str();
	glShaderSource(shaderID, 1, &src, NULL);

	GLint status;
	glCompileShader(shaderID);
	glGetShaderiv(shaderID, GL_COMPILE_STATUS, &status);
	if (status == GL_FALSE)
	{
		char log[256];
		glGetShaderInfoLog(shaderID, 256, NULL, log);
		std::cerr << "Shader compile error: " << log << std::endl;
	}

	shaders[numShaders++] = shaderID;
}

void Shader::LinkProgram()
{
	shaderProg = glCreateProgram();

	if (shaders[VERTEX_SHADER] != 0)
	{
		glAttachShader(shaderProg, shaders[VERTEX_SHADER]);
	}
	if (shaders[FRAGMENT_SHADER] != 0)
	{
		glAttachShader(shaderProg, shaders[FRAGMENT_SHADER]);
	}
	if (shaders[GEOMETRY_SHADER] != 0)
	{
		glAttachShader(shaderProg, shaders[GEOMETRY_SHADER]);
	}

	GLint status;
	glLinkProgram(shaderProg);
	glGetProgramiv(shaderProg, GL_LINK_STATUS, &status);
	if (status == GL_FALSE)
	{
		char log[256];
		glGetProgramInfoLog(shaderProg, 256, NULL, log);
		std::cerr << "Shader program link error: " << log << std::endl;
	}

	glDeleteShader(shaders[VERTEX_SHADER]);
	glDeleteShader(shaders[FRAGMENT_SHADER]);
	glDeleteShader(shaders[GEOMETRY_SHADER]);
}

void Shader::Use() const
{
	glUseProgram(shaderProg);
}

void Shader::UnUse() const
{
	glUseProgram(0);
}

void Shader::AddAttribute(const std::string& attribute)
{
	attributes[attribute] = glGetAttribLocation(shaderProg, attribute.c_str());
}

void Shader::AddUniform(const std::string& uniform)
{
	uniforms[uniform] = glGetUniformLocation(shaderProg, uniform.c_str());
}

GLuint Shader::GetAttributeLoc(const std::string& attribute)
{
	return attributes[attribute];
}

GLuint Shader::GetUniformLoc(const std::string& uniform)
{
	return uniforms[uniform];
}

void Shader::DeleteProgram() const
{
	glDeleteProgram(shaderProg);
}