#include <QtWidgets>

#include "dialog.h"

CarDialog::CarDialog(std::shared_ptr<AutoAuto> a_, QWidget *parent = 0) : QWidget(parent), a(a_){
	
	carList = new QListWidget(this);

	buttonBox = new QDialogButtonBox(QDialogButtonBox::Save
                                     | QDialogButtonBox::Discard);

    connect(buttonBox, SIGNAL(accepted()), this, SLOT(save()));
    connect(buttonBox, SIGNAL(rejected()), this, SLOT(discard()));

    QVBoxLayout *mainLayout = new QVBoxLayout;
    mainLayout->addWidget(carList);
    mainLayout->addWidget(buttonBox);
    filllist();

    setLayout(mainLayout);
    setWindowTitle(tr("Select Car"));
}

CarDialog::~CarDialog(){
	delete carList;
	delete saveButton;
	delete discardButton;
	delete mainLayout;
}

CarDialog::filllist(){
	auto carvec = a.getResults();
	for (auto const& car : carvec) {
		new QListWidgetItem(tr(car->getModel()), carList);
	}
	
}