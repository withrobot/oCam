#ifndef BOOLEAN_CONTROL_FORM_H
#define BOOLEAN_CONTROL_FORM_H

#include <QWidget>

namespace Ui {
class ControlFormBoolean;
}

class ControlFormBoolean : public QWidget
{
    Q_OBJECT

public:
    explicit ControlFormBoolean(const char* name, bool enable=true, QWidget *parent = 0);
    ~ControlFormBoolean();

    void set_value(const int value);
    int get_value();
    const char* get_name();
    bool value_changed();

    void set_enabled(bool flag);

    bool is_enabled() const;

private slots:
    void on_rbDisable_toggled(bool checked);
    void on_rbEnable_toggled(bool checked);

private:
    Ui::ControlFormBoolean *ui;
    bool changed;
};

#endif // BOOLEAN_CONTROL_FORM_H
