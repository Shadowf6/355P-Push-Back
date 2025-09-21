#include "screen.h"

lv_obj_t *lx, *ly, *lt;
lv_obj_t *curr, *desc, *next, *prev, *lnext, *lprev;
lv_obj_t *side, *bside, *lside;
lv_obj_t *skill, *bskill, *lskill;
lv_obj_t *test_distX, *test_distY;

void swapSide(lv_event_t* e) {
    red = !red; 
    char s[12]; snprintf(s, sizeof(s), "Color: %s", red ? "Red" : "Blue"); 
    lv_label_set_text(side, s); 
}

void swapMode(lv_event_t* e) {
    skills = !skills;
    char s[12]; snprintf(s, sizeof(s), "Mode: %s", skills ? "Skills" : "Match");
    lv_label_set_text(skill, s);
}

void nextAuton(lv_event_t* e) {
    auton = auton % total + 1; 
    char a[20]; snprintf(a, sizeof(a), "Selected Auton: %d", auton); 
    lv_label_set_text(curr, a); 
    lv_label_set_text(desc, descriptions[auton - 1]);
}

void prevAuton(lv_event_t* e) {
    auton = (auton + total - 2) % total + 1; 
    char a[20]; snprintf(a, sizeof(a), "Selected Auton: %d", auton); 
    lv_label_set_text(curr, a); 
    lv_label_set_text(desc, descriptions[auton - 1]); 
}

void createDisplay(lv_obj_t* screen) {
    lx = lv_label_create(screen); 
    lv_obj_align(lx, LV_ALIGN_TOP_LEFT, 20, 20);
    ly = lv_label_create(screen); 
    lv_obj_align(ly, LV_ALIGN_TOP_LEFT, 20, 50);
    lt = lv_label_create(screen); 
    lv_obj_align(lt, LV_ALIGN_TOP_LEFT, 20, 80);

    curr = lv_label_create(screen); 
    lv_obj_align(curr, LV_ALIGN_TOP_RIGHT, -50, 20);
    desc = lv_label_create(screen); 
    lv_obj_align(desc, LV_ALIGN_TOP_RIGHT, -50, 50);
    next = lv_button_create(screen);  
    lv_obj_align(next, LV_ALIGN_TOP_RIGHT, -50, 90); 
    lv_obj_set_style_bg_color(next, lv_color_hex(0x808080), 0); 
    lv_obj_add_event_cb(next, nextAuton, LV_EVENT_SHORT_CLICKED, nullptr);
    prev = lv_button_create(screen); 
    lv_obj_align(prev, LV_ALIGN_TOP_RIGHT, -120, 90); 
    lv_obj_set_style_bg_color(prev, lv_color_hex(0x808080), 0); 
    lv_obj_add_event_cb(prev, prevAuton, LV_EVENT_SHORT_CLICKED, nullptr);
    lnext = lv_label_create(next); 
    lv_label_set_text(lnext, ">");
    lprev = lv_label_create(prev); 
    lv_label_set_text(lprev, "<");

    side = lv_label_create(screen); 
    lv_obj_align(side, LV_ALIGN_TOP_RIGHT, -50, 150);
    bside = lv_button_create(screen); 
    lv_obj_align(bside, LV_ALIGN_TOP_RIGHT, -50, 180); 
    lv_obj_set_style_bg_color(bside, lv_color_hex(0x808080), 0); 
    lv_obj_add_event_cb(bside, swapSide, LV_EVENT_SHORT_CLICKED, nullptr);
    lside = lv_label_create(bside); 
    lv_label_set_text(lside, "Switch");

    skill = lv_label_create(screen);
    lv_obj_align(skill, LV_ALIGN_TOP_RIGHT, -150, 150);
    bskill = lv_button_create(screen);
    lv_obj_align(bskill, LV_ALIGN_TOP_RIGHT, -150, 180);
    lv_obj_set_style_bg_color(bskill, lv_color_hex(0x808080), 0);
    lv_obj_add_event_cb(bskill, swapMode, LV_EVENT_SHORT_CLICKED, nullptr);
    lskill = lv_label_create(bskill);
    lv_label_set_text(lskill, "Switch");

    char a[20]; snprintf(a, sizeof(a), "Selected Auton: %d", auton); 
    lv_label_set_text(curr, a); 
    lv_label_set_text(desc, descriptions[auton - 1]);
    char c[20]; snprintf(c, sizeof(c), "Color: %s", red ? "Red" : "Blue"); 
    lv_label_set_text(side, c);
    char s[20]; snprintf(s, sizeof(s), "Mode: %s", skills ? "Skills" : "Match");
    lv_label_set_text(skill, s);

    /*test_distX = lv_label_create(screen); 
    lv_obj_align(test_distX, LV_ALIGN_TOP_LEFT, 20, 130);
    test_distY = lv_label_create(screen); 
    lv_obj_align(test_distY, LV_ALIGN_TOP_LEFT, 20, 160);*/
}

void updateCoords(float x, float y, float t) {
    char xx[20], yy[20], tt[20];
    snprintf(xx, sizeof(xx), "X: %.3f", x); 
    lv_label_set_text(lx, xx); 
    snprintf(yy, sizeof(yy), "Y: %.3f", y); 
    lv_label_set_text(ly, yy); 
    snprintf(tt, sizeof(tt), "Theta: %.3f", t); 
    lv_label_set_text(lt, tt);
}

void updateDist(float x, float y) {
    char a[20]; snprintf(a, sizeof(a), "Distance X: %.3f", x); 
    lv_label_set_text(test_distX, a);
    char b[20]; snprintf(b, sizeof(b), "Distance Y: %.3f", y);
    lv_label_set_text(test_distY, b);
}
