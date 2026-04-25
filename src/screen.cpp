#include "screen.h"

lv_obj_t *lx, *ly, *lt;
lv_obj_t *dl, *dr, *df, *db;

lv_obj_t *curr, *desc, *next, *prev, *lnext, *lprev;
lv_obj_t *sto, *sti, *stdist, *stm;
lv_obj_t *vnext, *vlnext;
lv_obj_t *bmode, *lmode;

void nextAuton(lv_event_t* e) {
    auton = auton % total + 1; 
    variation = 1;
    lv_label_set_text_fmt(curr, "Selected Auton: %d", auton); 
    lv_label_set_text(desc, descriptions[auton - 1]);
    lv_label_set_text(vlnext, variations[auton - 1][variation - 1]);
}

void prevAuton(lv_event_t* e) {
    auton = (auton + total - 2) % total + 1; 
    variation = 1;
    lv_label_set_text_fmt(curr, "Selected Auton: %d", auton); 
    lv_label_set_text(desc, descriptions[auton - 1]);
    lv_label_set_text(vlnext, variations[auton - 1][variation - 1]);
}

void switchVar(lv_event_t* e) {
    variation = variation % variations[auton - 1].size() + 1;
    lv_label_set_text(vlnext, variations[auton - 1][variation - 1]);
}

void switchMode(lv_event_t* e) {
    skills = !skills;
    lv_obj_set_style_bg_color(bmode, skills ? lv_color_hex(0x6495ED) : lv_color_hex(0xED2939), 0);
    lv_label_set_text_fmt(lmode, "Mode: %s", skills ? "Skills" : "Match");
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

    df = lv_label_create(screen);
    lv_obj_align(df, LV_ALIGN_TOP_LEFT, 20, 190);

    db = lv_label_create(screen);
    lv_obj_align(db, LV_ALIGN_TOP_LEFT, 20, 220);


    curr = lv_label_create(screen); 
    lv_obj_align(curr, LV_ALIGN_TOP_RIGHT, -20, 20);
    lv_label_set_text_fmt(curr, "Selected Auton: %d", auton); 

    desc = lv_label_create(screen); 
    lv_obj_align(desc, LV_ALIGN_TOP_RIGHT, -20, 50);
    lv_label_set_text(desc, descriptions[auton - 1]);

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

    sto = lv_label_create(screen);
    lv_obj_align(sto, LV_ALIGN_TOP_RIGHT, -20, 150);

    sti = lv_label_create(screen);
    lv_obj_align(sti, LV_ALIGN_TOP_RIGHT, -20, 170);

    stdist = lv_label_create(screen);
    lv_obj_align(stdist, LV_ALIGN_TOP_RIGHT, -20, 190);

    stm = lv_label_create(screen);
    lv_obj_align(stm, LV_ALIGN_TOP_RIGHT, -20, 210);


    vnext = lv_button_create(screen);
    lv_obj_align(vnext, LV_ALIGN_TOP_RIGHT, -130, 80);
    lv_obj_set_style_bg_color(vnext, lv_color_hex(0x6082B6), 0);
    lv_obj_add_event_cb(vnext, switchVar, LV_EVENT_SHORT_CLICKED, nullptr);

    vlnext = lv_label_create(vnext);
    lv_label_set_text(vlnext, variations[auton - 1][0]);

    bmode = lv_button_create(screen);
    lv_obj_align(bmode, LV_ALIGN_TOP_MID, 0, 20);
    lv_obj_set_style_bg_color(bmode, skills ? lv_color_hex(0x6495ED) : lv_color_hex(0xED2939), 0);
    lv_obj_add_event_cb(bmode, switchMode, LV_EVENT_SHORT_CLICKED, nullptr);

    lmode = lv_label_create(bmode);
    lv_label_set_text_fmt(lmode, "Mode: %s", skills ? "Skills" : "Match");    
}

void setStatus(bool r, bool i, bool d, bool m) {
    lv_label_set_text_fmt(sto, "Rotation sensors plugged in? %s", r ? "Yes" : "No");
    lv_label_set_text_fmt(sti, "IMU plugged in? %s", i ? "Yes" : "No");
    lv_label_set_text_fmt(stdist, "Distance sensors plugged in? %s", d ? "Yes" : "No");
    lv_label_set_text_fmt(stm, "Intake motors plugged in? %s", m ? "Yes" : "No");
}

void updateCoords(float x, float y, float t, float l, float r, float f, float b) {
    char bf[20];

    snprintf(bf, sizeof(bf), "X: %.2f", x);
    lv_label_set_text(lx, bf);

    snprintf(bf, sizeof(bf), "Y: %.2f", y);
    lv_label_set_text(ly, bf);

    snprintf(bf, sizeof(bf), "Theta: %.2f", t);
    lv_label_set_text(lt, bf);

    snprintf(bf, sizeof(bf), "L: %.2f", l);
    lv_label_set_text(dl, bf);

    snprintf(bf, sizeof(bf), "R: %.2f", r);
    lv_label_set_text(dr, bf);

    snprintf(bf, sizeof(bf), "F: %.2f", f);
    lv_label_set_text(df, bf);

    snprintf(bf, sizeof(bf), "B: %.2f", b);
    lv_label_set_text(db, bf);
}
