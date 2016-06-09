#include "controlform_boolean.h"
#include "ui_controlform_boolean.h"

ControlFormBoolean::ControlFormBoolean(const char* name, bool enable, QWidget *parent) :
    QWidget(parent), ui(new Ui::ControlFormBoolean), changed(false)
{
    ui->setupUi(this);
    ui->lblControlName->setText(tr(name));

    set_enabled(enable);
}

ControlFormBoolean::~ControlFormBoolean()
{
    delete ui;
}

const char* ControlFormBoolean::get_name()
{
    return ui->lblControlName->text().toStdString().c_str();
}

int ControlFormBoolean::get_value()
{
    changed = false;
    return (int)ui->rbEnable->isChecked();
}

void ControlFormBoolean::set_value(const int value)
{
    ui->rbEnable->setChecked((bool)value);
    ui->rbDisable->setChecked(!(bool)value);
}

bool ControlFormBoolean::value_changed()
{    
    return changed;
}

void ControlFormBoolean::on_rbDisable_toggled(bool checked)
{
    ui->rbEnable->setChecked(!checked);
    changed = true;
}

void ControlFormBoolean::on_rbEnable_toggled(bool checked)
{
    ui->rbDisable->setChecked(!checked);
    changed = true;
}

void ControlFormBoolean::set_enabled(bool flag)
{
    ui->lblControlName->setEnabled(flag);
    ui->rbDisable->setEnabled(flag);
    ui->rbEnable->setEnabled(flag);
}

bool ControlFormBoolean::is_enabled() const
{
    return ui->lblControlName->isEnabled();
}
