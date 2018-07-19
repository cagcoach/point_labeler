#version 330 core

layout (location = 0) in vec3  in_vertex;


uniform uint scan;

uniform mat4 pose;

uniform float minRange;
uniform float maxRange;

uniform vec2 tilePos;
uniform float tileSize;
uniform float tileBoundary;

uniform sampler2D texImage;
uniform sampler2D texLabel;

uniform mat4 Tr;
uniform mat4x3 P2;

uniform float width;
uniform float height;

out POINT
{
  bool valid;
  vec3 color;
  int label;
} vs_out;


void main()
{
  float range = length(in_vertex);
  
  vec4 v_global = pose * vec4(in_vertex, 1.0);
  vec2 v = v_global.xy - tilePos;
  
  bool inside_tile = (abs(v.x) < 0.5 * (tileSize + tileBoundary) && abs(v.y) < 0.5 * (tileSize + tileBoundary));
  bool out_of_range = range < minRange || range > maxRange;
   
  vs_out.valid = false;
  
  if(inside_tile && !out_of_range)
  {
    vs_out.valid = true;

    vec4 coords_camera = Tr * vec4(in_vertex, 1);
    vec3 coords = P2 * (Tr * vec4(in_vertex, 1));
    coords = vec3((coords.x / coords.z) / width, (height - coords.y / coords.z) / height, 0);
    
    if(coords_camera.z < 0 && coords.x < 0 && coords.y < 0 && coords.x > width &&  coords.y > height)
    {
      
    }
    else
    {
      int label = int(texture(texImage, coords.xy).r);
      
      vs_out.label = 0;
      if(label == 0) vs_out.label = 40;
      if(label == 1) vs_out.label = 48;
      if(label == 2) vs_out.label = 50;
      if(label == 4) vs_out.label = 51;
      if(label == 13) vs_out.label = 10;
      if(label == 8) vs_out.label = 70;
      if(label == 11) vs_out.label = 30;
      if(label == 18) vs_out.label = 11;
      if(label == 17) vs_out.label = 15;
      
      vs_out.color = texture(texLabel, coords.xy).rgb;
    }
  }
}