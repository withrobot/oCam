#include "controlform_menu.h"
#include "ui_controlform_menu.h"

ControlFormMenu::ControlFormMenu(const char* name, std::vector<Withrobot::camera_control_menu> menu_list, bool enable, QWidget *parent) :
    QWidget(parent), ui(new Ui::ControlFormMenu), changed(false)
{
    ui->setupUi(this);

    ui->lblControlName->setText(QString(name));
    for (unsigned int i=0; i < menu_list.size(); i++) {
        ui->cbControls->addItem(QString((const char*)menu_list[i].name));
    }

    set_enabled(enable);
}

ControlFormMenu::~ControlFormMenu()
{
    delete ui;
}


void ControlFormMenu::set_value(const int value)
{
    ui->cbControls->setCurrentIndex(value);
}


bool ControlFormMenu::value_changed() {
    return changed;
}

const char* ControlFormMenu::get_name()
{
    return ui->lblControlName->text().toStdString().c_str();
}

int ControlFormMenu::get_value()
{
    changed = false;
    return ui->cbControls->currentIndex();
}

void ControlFormMenu::on_cbControls_currentIndexChanged(int index)
{
    ui->cbControls->setCurrentIndex(index);
    changed = true;
}

void ControlFormMenu::set_enabled(bool flag)
{
    ui->lblControlName->setEnabled(flag);
    ui->cbControls->setEnabled(flag);
}

bool ControlFormMenu::is_enabled() const
{
    return ui->lblControlName->isEnabled();
}
