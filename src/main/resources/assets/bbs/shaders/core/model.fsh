#version 330

#moj_import <minecraft:fog.glsl>
#moj_import <minecraft:dynamictransforms.glsl>

uniform sampler2D Sampler0;

in float sphericalVertexDistance;
in float cylindricalVertexDistance;
in vec4 vertexColor;
in vec4 lightMapColor;
in vec4 overlayColor;
in vec2 texCoord0;

out vec4 fragColor;

/* TODO(1.21.11 render): two-pass translucency. On 1.21.1 this shader took a loose
 * `uniform int PassMode` that split the draw — pass 1 kept only the opaque texels (they write
 * depth), pass 2 kept only the semi-transparent ones and replayed them sorted far-to-near from
 * FormTranslucentQueue. 1.21.5+ has no mutable GlUniforms (a custom uniform must ride a std140
 * UBO declared on the RenderPipeline) and no VertexBuffer/BufferRenderer for the deferred
 * replay, so the queue is disabled on this branch and this shader draws single-pass.
 * Re-adding it needs a PassMode UBO entry plus one pipeline variant per pass. */

void main()
{
    vec4 color = texture(Sampler0, texCoord0);

    if (color.a < 0.1)
    {
        discard;
    }

    color *= vertexColor * ColorModulator;

    color.rgb = mix(overlayColor.rgb, color.rgb, overlayColor.a);
    color *= lightMapColor;

    fragColor = apply_fog(color, sphericalVertexDistance, cylindricalVertexDistance,
        FogEnvironmentalStart, FogEnvironmentalEnd, FogRenderDistanceStart, FogRenderDistanceEnd, FogColor);
}
