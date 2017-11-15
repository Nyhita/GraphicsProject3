R"zzz(
#version 330 core
uniform vec4 light_position;
uniform vec3 camera_position;
uniform mat4[127] Ds;
uniform mat4[127] Us;
in vec4 vertex_position;
in vec4 normal;
in vec2 uv;

in vec4 weights1;
in vec4 weights2;
in vec4 weights3;

in vec4 bone_ids1;
in vec4 bone_ids2;
in vec4 bone_ids3;

out vec4 vs_light_direction;
out vec4 vs_normal;
out vec2 vs_uv;
out vec4 vs_camera_direction;
void main() {
	gl_Position = vec4(0.0, 0.0, 0.0, 0.0);
	int n = 0;
	//float added_weight = 0.0;

	//for (n = 0; n < 4; n++) {
		// weights[3][2]
		//if(weights[n/3][n%3] <= 0.0)
		//if(weights1[n] <= 0.0)
		//	continue;
		//gl_Position += weights[n/3][n%3] * Ds[int(bone_ids[n/3][n%3])] * inverse(Us[int(bone_ids[n/3][n%3])]) * vertex_position; 
		//gl_Position += weights1[n] * Ds[int(bone_ids1[n])] * inverse(Us[int(bone_ids1[n])]) * vertex_position; 
		//added_weight += weights[n/3][n%3];
	//}

	for (n = 0; n < 4; n++) {
		if(weights1[n] <= 0.0)
			continue;
		gl_Position += weights1[n] * Ds[int(bone_ids1[n])] * inverse(Us[int(bone_ids1[n])]) * vertex_position; 
	}

	for (n = 0; n < 4; n++) {
		if(weights2[n] <= 0.0)
			continue;
		gl_Position += weights2[n] * Ds[int(bone_ids2[n])] * inverse(Us[int(bone_ids2[n])]) * vertex_position; 
	}

	for (n = 0; n < 4; n++) {
		if(weights3[n] <= 0.0)
			continue;
		gl_Position += weights3[n] * Ds[int(bone_ids3[n])] * inverse(Us[int(bone_ids3[n])]) * vertex_position; 
	}

	vs_light_direction = light_position - gl_Position;
	vs_camera_direction = vec4(camera_position, 1.0) - gl_Position;
	vs_normal = normal;
	vs_uv = uv;
}
)zzz"
