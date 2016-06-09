#include "format_tree_form.h"
#include "ui_format_tree_form.h"

#include "withrobot_utility.hpp"
#include "withrobot_debug_print.h"

FormatTreeForm::FormatTreeForm(QString name, QWidget *parent) :
    QWidget(parent), ui(new Ui::FormatTreeForm), changed(false)
{
    ui->setupUi(this);

    /* set the number of columns in the tree */
    ui->treeFormat->setColumnCount(1);
    ui->treeFormat->setHeaderLabel(name);
}

FormatTreeForm::~FormatTreeForm()
{
    delete ui;
}

void FormatTreeForm::change_title(std::string str)
{
    ui->treeFormat->setHeaderLabel(str.c_str());
}

std::string FormatTreeForm::get_format_name()
{
    QList<QTreeWidgetItem*> selected_item_list = ui->treeFormat->selectedItems();

    std::string retval;
    retval.clear();

    std::string top_level_text;
    std::string item_text;

    for (int i=0; i < selected_item_list.size(); i++) {
        QTreeWidgetItem* parent_item = selected_item_list[i]->parent();

        if (!parent_item) {
            return retval;
        }

        top_level_text = parent_item->text(0).toStdString();
        item_text = selected_item_list[i]->text(0).toStdString();
        retval = top_level_text + item_text;
    }

    changed = false;

    return retval;
}


void FormatTreeForm::add_format_list(const char* format_name,
                                     std::vector<const char *>& frame_rate_list)
{
    add_format_tree(format_name, frame_rate_list);
}

void FormatTreeForm::add_format_tree(QString format_name,
                                     std::vector<const char *>& frame_rate_list)
{
    // QTreeWidgetItem(QTreeWidget* parent, int type = Type)
    QTreeWidgetItem* tree_item = new QTreeWidgetItem(ui->treeFormat);

    // QTreeWidgetIten::setText(int column, const QString& text)
    tree_item->setText(0, format_name);

    std::string str;

    for (unsigned int i=0; i < frame_rate_list.size(); i++) {
        str = frame_rate_list[i];
        str.erase(str.find(format_name.toStdString()), format_name.size());
        add_rate_tree(tree_item, str.c_str());
    }
}

void FormatTreeForm::add_rate_tree(QTreeWidgetItem* parent, QString frame_rate_name)
{
    QTreeWidgetItem* tree_item = new QTreeWidgetItem();

    tree_item->setText(0, frame_rate_name);

    parent->addChild(tree_item);
}

void FormatTreeForm::on_btnApply_clicked()
{
    changed = true;
}
