#include "controlform_pushbtn.h"
#include "ui_controlform_pushbtn.h"

ControlFormPushBtn::ControlFormPushBtn(const char* name, const char* btnName, QWidget *parent) :
    QWidget(parent), ui(new Ui::ControlFormPushBtn)
{
    ui->setupUi(this);

    ui->lblControlName->setText(QString(name));
    ui->pushButton->setText(QString(btnName));
}

ControlFormPushBtn::~ControlFormPushBtn()
{
    delete ui;
}

void ControlFormPushBtn::on_pushButton_clicked()
{
    emit btnClicked(true);
}
