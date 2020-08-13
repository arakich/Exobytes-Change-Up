#include "main.h"

lv_obj_t* createBtn(lv_obj_t * parent, lv_coord_t x, lv_coord_t y, lv_coord_t width, lv_coord_t height, int id, const char* title)
{
  parent = lv_btn_create(lv_scr_act(), NULL);
  lv_obj_set_pos(parent, x, y);
  lv_obj_set_size(parent,width,height);
  lv_obj_set_free_num(parent, id);

  lv_obj_t * label = lv_label_create(parent, NULL);
  lv_label_set_text(label, title);

  return parent;
}

void createTextLabel(lv_obj_t * parent, const char* text, lv_coord_t x, lv_coord_t y)
{
  parent = lv_label_create(lv_scr_act(), NULL);
  lv_label_set_text(parent,text);
  lv_obj_set_pos(parent,x,y);
}

void initialize()
{
  pros::Task task_5 (thread_sensors, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "");
  pros::Task task_1 (thread_conveyor, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "");
  pros::Task task_2 (thread_intake, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "");
  pros::Task task_6 (thread_Odometry, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "");
}

void disabled()
{

}

void competition_initialize()
{

}

void autonomous()
{

}

void opcontrol()
{
  pros::Task task_3 (thread_drive, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "");
  pros::Task task_4 (thread_control, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "");
}
