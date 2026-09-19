/* Host-only audit. preset_snapshot.h is extracted from seize_sky.h by PowerShell.
 * Link this file with the project's unmodified User/src/kinematics.c.
 */
#include "kinematics.h"
#include "preset_snapshot.h"

#define POINT(n) {#n, {ARM_U1_##n##_POS, ARM_U2_##n##_POS}, ARM_DJ_##n##_POS}
typedef struct { const char *name; Unitree_Theta_t u; float dj; } Point;
static const Point points[] = {
    POINT(START), POINT(READY), POINT(SKY_READY), POINT(KEEP), POINT(SKY),
    POINT(LOW), POINT(LOW1), POINT(MID), POINT(MID1), POINT(HIGH)
};

int main(void)
{
    float max_angle = 0, max_pos = 0;
    unsigned failures = 0, count = 0;
    puts("state,u1_rad,u2_rad,dj_deg,x_mm,y_mm,theta2_deg,roundtrip_rad");
    for (unsigned i = 0; i < sizeof(points)/sizeof(points[0]); ++i) {
        Vec2 p = Forward(points[i].u);
        Unitree_Theta_t back = Inverse(p);
        float err = fmaxf(fabsf(back.u1_theta-points[i].u.u1_theta),
                          fabsf(back.u2_theta-points[i].u.u2_theta));
        printf("%s,%.6f,%.6f,%.3f,%.3f,%.3f,%.3f,%.9f\n", points[i].name,
            points[i].u.u1_theta, points[i].u.u2_theta, points[i].dj,
            p.x, p.y, U2_Motor2Geom(points[i].u.u1_theta, points[i].u.u2_theta)*180/PI, err);
        if (err > 0.0001f) ++failures;
    }
    /* Positive-elbow branch, away from folded/extended singularities and atan2 seam. */
    for (int i=0; i<=100; ++i) for (int j=0; j<=100; ++j) {
        float t1 = -1.4f + i*0.028f, t2 = 0.15f + j*0.026f;
        Unitree_Theta_t u = {t1-ARM_U1_ZERO_POS, 0};
        u.u2_theta = (ARM_U2_ZERO_POS-t2)/ARM_U2_RATIO-u.u1_theta;
        Vec2 p = Forward(u);
        Unitree_Theta_t back = Inverse(p);
        Vec2 pb = Forward(back);
        float ea = fmaxf(fabsf(back.u1_theta-u.u1_theta), fabsf(back.u2_theta-u.u2_theta));
        float ep = hypotf(pb.x-p.x, pb.y-p.y);
        max_angle = fmaxf(max_angle, ea); max_pos = fmaxf(max_pos, ep);
        if (!isfinite(ea) || !isfinite(ep) || ea>0.0001f || ep>0.01f) ++failures;
        ++count;
    }
    printf("GRID count=%u max_angle_rad=%.9f max_position_mm=%.9f failures=%u\n", count,max_angle,max_pos,failures);
    Vec2 bad[] = {{0,0},{900,0}};
    for (unsigned i=0;i<2;++i) {
        Vec2 actual=Forward(Inverse(bad[i]));
        printf("UNREACHABLE requested=(%.1f,%.1f) returned=(%.3f,%.3f) error_mm=%.3f\n",
            bad[i].x,bad[i].y,actual.x,actual.y,hypotf(actual.x-bad[i].x,actual.y-bad[i].y));
    }
    Unitree_Theta_t neg = {0.5f, (ARM_U2_ZERO_POS+0.5f)/ARM_U2_RATIO-0.5f};
    Unitree_Theta_t other = Inverse(Forward(neg));
    printf("NEGATIVE_ELBOW before=(%.6f,%.6f) after=(%.6f,%.6f)\n",neg.u1_theta,neg.u2_theta,other.u1_theta,other.u2_theta);
    return failures ? 1 : 0;
}
