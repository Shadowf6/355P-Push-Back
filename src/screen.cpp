#include "screen.h"

lv_obj_t *lx, *ly, *lt;
lv_obj_t *dl, *dr, *db;
lv_obj_t *curr, *desc, *next, *prev, *lnext, *lprev;
lv_obj_t *sto, *sti, *stdist, *stm;

void nextAuton(lv_event_t* e) {
    auton = auton % total + 1; 
    lv_label_set_text_fmt(curr, "Selected Auton: %d", auton); 
    lv_label_set_text(desc, descriptions[auton - 1]);
}

void prevAuton(lv_event_t* e) {
    auton = (auton + total - 2) % total + 1; 
    lv_label_set_text_fmt(curr, "Selected Auton: %d", auton); 
    lv_label_set_text(desc, descriptions[auton - 1]);
}

void createDisplay(lv_obj_t* screen) {
    lx = lv_label_create(screen); 
    lv_obj_align(lx, LV_ALIGN_TOP_LEFT, 20, 20);

    ly = lv_label_create(screen); 
    lv_obj_align(ly, LV_ALIGN_TOP_LEFT, 20, 50);

    lt = lv_label_create(screen); 
    lv_obj_align(lt, LV_ALIGN_TOP_LEFT, 20, 80);

    dl = lv_label_create(screen);
    lv_obj_align(dl, LV_ALIGN_TOP_LEFT, 20, 130);

    dr = lv_label_create(screen);
    lv_obj_align(dr, LV_ALIGN_TOP_LEFT, 20, 160);

    db = lv_label_create(screen);
    lv_obj_align(db, LV_ALIGN_TOP_LEFT, 20, 190);

    curr = lv_label_create(screen); 
    lv_obj_align(curr, LV_ALIGN_TOP_RIGHT, -20, 20);

    desc = lv_label_create(screen); 
    lv_obj_align(desc, LV_ALIGN_TOP_RIGHT, -20, 50);

    next = lv_button_create(screen);  
    lv_obj_align(next, LV_ALIGN_TOP_RIGHT, -20, 80); 
    lv_obj_set_style_bg_color(next, lv_color_hex(0x6082B6), 0); 
    lv_obj_add_event_cb(next, nextAuton, LV_EVENT_SHORT_CLICKED, nullptr);

    prev = lv_button_create(screen); 
    lv_obj_align(prev, LV_ALIGN_TOP_RIGHT, -70, 80); 
    lv_obj_set_style_bg_color(prev, lv_color_hex(0x6082B6), 0); 
    lv_obj_add_event_cb(prev, prevAuton, LV_EVENT_SHORT_CLICKED, nullptr);

    lnext = lv_label_create(next); 
    lv_label_set_text(lnext, ">");

    lprev = lv_label_create(prev); 
    lv_label_set_text(lprev, "<");

    lv_label_set_text_fmt(curr, "Selected Auton: %d", auton); 
    lv_label_set_text(desc, descriptions[auton - 1]);

    sto = lv_label_create(screen);
    lv_obj_align(sto, LV_ALIGN_TOP_RIGHT, -20, 150);

    sti = lv_label_create(screen);
    lv_obj_align(sti, LV_ALIGN_TOP_RIGHT, -20, 170);

    stdist = lv_label_create(screen);
    lv_obj_align(stdist, LV_ALIGN_TOP_RIGHT, -20, 190);

    stm = lv_label_create(screen);
    lv_obj_align(stm, LV_ALIGN_TOP_RIGHT, -20, 210);
}

void updateStatus(bool r, bool i, bool d, bool m) {
    lv_label_set_text_fmt(sto, "Rotation sensors plugged in? %s", r ? "Yes" : "No");
    lv_label_set_text_fmt(sti, "IMU plugged in? %s", i ? "Yes" : "No");
    lv_label_set_text_fmt(stdist, "Distance sensors plugged in? %s", d ? "Yes" : "No");
    lv_label_set_text_fmt(stm, "Intake motors plugged in? %s", m ? "Yes" : "No");
}

void updateCoords(float x, float y, float t, float l, float r, float b) {
    char xx[20]; snprintf(xx, sizeof(xx), "X: %.3f", x); 
    lv_label_set_text(lx, xx); 

    char yy[20]; snprintf(yy, sizeof(yy), "Y: %.3f", y); 
    lv_label_set_text(ly, yy); 

    char tt[20]; snprintf(tt, sizeof(tt), "Theta: %.3f", t); 
    lv_label_set_text(lt, tt);

    char ll[20]; snprintf(ll, sizeof(ll), "L: %.1f", l); 
    lv_label_set_text(dl, ll); 

    char rr[20]; snprintf(rr, sizeof(rr), "R: %.1f", r); 
    lv_label_set_text(dr, rr); 

    char bb[20]; snprintf(bb, sizeof(bb), "B: %.1f", b); 
    lv_label_set_text(db, bb);
}
