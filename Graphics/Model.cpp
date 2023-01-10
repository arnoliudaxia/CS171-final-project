
#include "Model.h"
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#define POINT_SIZE 5.0f

Model::Model(const std::vector<glm::vec3>& vertices, const std::vector<int>& indices)
	: vertices(vertices), indices(indices), primType(GL_TRIANGLES), color(0.4, 0.9, 0.1), lineWidth(3.0f)
{
	shader.LoadFromFile(GL_VERTEX_SHADER, "Graphics/shader.vert");
	shader.LoadFromFile(GL_FRAGMENT_SHADER, "Graphics/shader.frag");
	shader.LinkProgram();
	shader.Use();
	shader.AddAttribute("vVertex");
	shader.AddUniform("MVP");
	shader.AddUniform("vColor");
	glUniform3fv(shader.GetUniformLoc("vColor"), 1, glm::value_ptr(color));
	shader.UnUse();

	// setup vao and vbo stuff
	// generate vao and vbo
	if (!vaoID)
		glGenVertexArrays(1, &vaoID);
	if (!vboVerticesID)
		glGenBuffers(1, &vboVerticesID);
	if (!vboIndicesID)
		glGenBuffers(1, &vboIndicesID);

	// allocate buffer data
	glBindVertexArray(vaoID);

	glBindBuffer(GL_ARRAY_BUFFER, vboVerticesID);
	glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(glm::vec3), 0, GL_STATIC_DRAW);

	GLfloat* pBuffer = static_cast<GLfloat*>(glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY));
	FillVertexBuffer(pBuffer);
	glUnmapBuffer(GL_ARRAY_BUFFER);

	glEnableVertexAttribArray(shader.GetAttributeLoc("vVertex"));
	glVertexAttribPointer(shader.GetAttributeLoc("vVertex"), 3, GL_FLOAT, GL_FALSE, 0, 0);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vboIndicesID);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLuint), 0, GL_STATIC_DRAW);

	GLuint* pIBuffer = static_cast<GLuint*>(glMapBuffer(GL_ELEMENT_ARRAY_BUFFER, GL_WRITE_ONLY));
	FillIndexBuffer(pIBuffer);
	glUnmapBuffer(GL_ELEMENT_ARRAY_BUFFER);

	glBindVertexArray(0);
}

Model::~Model()
{
	//Destroy shader
	shader.DeleteProgram();

	//Destroy vao and vbo
	glDeleteBuffers(1, &vboVerticesID);
	glDeleteBuffers(1, &vboIndicesID);
	glDeleteVertexArrays(1, &vaoID);
}

void Model::SetPrimitive(GLenum type)
{
	primType = type;
}

void Model::FillVertexBuffer(GLfloat* pBuffer)
{
	glm::vec3* vertices = (glm::vec3*)(pBuffer);

	for (glm::vec3 v : this->vertices)
		*vertices++ = v;
}

void Model::FillIndexBuffer(GLuint* pBuffer)
{
	//fill indices array
	GLuint* id = pBuffer;

	for (int i : indices)
		*id++ = i;
}

void Model::Render()
{
	shader.Use();
	if (glm::value_ptr(MVP) != 0)
		glUniformMatrix4fv(shader.GetUniformLoc("MVP"), 1, GL_FALSE, glm::value_ptr(MVP));
	glBindVertexArray(vaoID);
	glLineWidth(lineWidth);
	glPointSize(POINT_SIZE);
	glDrawElements(primType, indices.size(), GL_UNSIGNED_INT, 0);
	glBindVertexArray(0);
	shader.UnUse();
}

void Model::SetColor(const glm::vec3 color)
{
	shader.Use();
	this->color = color;
	glUniform3fv(shader.GetUniformLoc("vColor"), 1, glm::value_ptr(color));
	shader.UnUse();
}

void Model::SetLineWidth(const float width)
{
	lineWidth = width;
}

void Model::SetMVP(const glm::mat4& MVP)
{
	this->MVP = MVP;
}

glm::mat4 Model::GetMVP() const
{
	return MVP;
}