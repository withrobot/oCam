#ifndef INTEGER_CONTROL_FORM_H
#define INTEGER_CONTROL_FORM_H

#include <QWidget>

#include "withrobot_camera.hpp"

namespace Ui {
class ControlFormInteger;
}

class ControlFormInteger : public QWidget
{
    Q_OBJECT

public:
    explicit ControlFormInteger(const char* name, int min, int max, int step=1, bool enable=true, QWidget *parent = 0);
    ~ControlFormInteger();

    int get_value();
    const char* get_name();
    void set_value(const int value);
    bool value_changed();

    void set_enabled(bool flag);

    bool is_enabled() const;

private slots:
    void on_hsValue_valueChanged(int value);

    void on_spnbxValue_valueChanged(int arg1);

private:
    Ui::ControlFormInteger *ui;
    bool changed;
    int step;
};

#endif // INTEGER_CONTROL_FORM_H
