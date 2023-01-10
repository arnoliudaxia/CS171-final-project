
#version 400

out vec4 vFragColor;

uniform vec3 vColor;

void main()
{
	vFragColor = vec4(vColor,1.0f);
}