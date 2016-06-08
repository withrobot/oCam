#ifndef MENU_CONTROL_FORM_H
#define MENU_CONTROL_FORM_H

#include <QWidget>

#include "withrobot_camera.hpp"

namespace Ui {
class ControlFormMenu;
}

class ControlFormMenu : public QWidget
{
    Q_OBJECT

public:
    explicit ControlFormMenu(const char* name, std::vector<Withrobot::camera_control_menu> menu_list, bool enable=true, QWidget *parent = 0);
    ~ControlFormMenu();

    void set_value(const int value);

    bool value_changed();
    int get_value();
    const char* get_name();

    void set_enabled(bool flag);

    bool is_enabled() const;

private slots:
    void on_cbControls_currentIndexChanged(int index);

private:
    Ui::ControlFormMenu *ui;

    bool changed;
};

#endif // MENU_CONTROL_FORM_H
