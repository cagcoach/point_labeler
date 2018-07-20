#version 330 core

layout(points) in;
layout(points, max_vertices = 1) out;

in POINT 
{
  bool valid;
  vec3 color;
  uint label;
} gs_in[];


out vec3  out_color;
out uint  out_label;


void main()
{
  if(gs_in[0].valid)
  {
    out_color = gs_in[0].color;
    out_label = uint(gs_in[0].label);

    EmitVertex();
    EndPrimitive();  
  }
  
}