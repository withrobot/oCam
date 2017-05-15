#include "controlform_integer.h"
#include "ui_controlform_integer.h"

ControlFormInteger::ControlFormInteger(const char* name, int min, int max, int step, bool enable, QWidget *parent) :
    QWidget(parent), ui(new Ui::ControlFormInteger), changed(false), step(step)
{
    ui->setupUi(this);
    ui->lblControlName->setText(tr(name));
    ui->hsValue->setMinimum(min);
    ui->hsValue->setMaximum(max);
    ui->hsValue->setTickInterval(step);
    ui->spnbxValue->setMinimum(min);
    ui->spnbxValue->setMaximum(max);
    ui->spnbxValue->setSingleStep(step);
    ui->lblMin->setText(QString::number(min));
    ui->lblMax->setText(QString::number(max));

    set_enabled(enable);
}

ControlFormInteger::~ControlFormInteger()
{
    delete ui;
}

bool ControlFormInteger::value_changed() {
    return changed;
}

const char* ControlFormInteger::get_name()
{
    //return ui->lblControlName->text().toStdString().c_str();      // fix the locale issue -20160801 gnohead
    return ui->lblControlName->text().toLocal8Bit().constData();
}

int ControlFormInteger::get_value()
{
    changed = false;
    return ui->spnbxValue->value();
}

void ControlFormInteger::set_value(const int value)
{
    ui->spnbxValue->setValue(value);
    ui->hsValue->setValue(value);
}

void ControlFormInteger::on_hsValue_valueChanged(int value)
{
    ui->spnbxValue->setValue(value);
    changed = true;
}

void ControlFormInteger::on_spnbxValue_valueChanged(int arg1)
{
    ui->hsValue->setValue(arg1);
    changed = true;
}

void ControlFormInteger::set_enabled(bool flag)
{
    ui->lblControlName->setEnabled(flag);
    ui->hsValue->setEnabled(flag);
    ui->spnbxValue->setEnabled(flag);
    ui->lblMin->setEnabled(flag);
    ui->lblMax->setEnabled(flag);
}

bool ControlFormInteger::is_enabled() const
{
    return ui->lblControlName->isEnabled();
}
