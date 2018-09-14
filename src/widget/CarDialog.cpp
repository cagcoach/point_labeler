#include <QtWidgets>

#include "CarDialog.h"

CarDialog::CarDialog(std::shared_ptr<AutoAuto> a_, QWidget* parent) : a(a_),QDialog(parent){
	
	carList = new QListWidget(this);

	buttonBox = new QDialogButtonBox(QDialogButtonBox::Save
                                     | QDialogButtonBox::Cancel);

    connect(buttonBox, SIGNAL(accepted()), this, SLOT(save()));
    connect(buttonBox, SIGNAL(rejected()), this, SLOT(discard()));
    connect(carList, SIGNAL(itemSelectionChanged()), this, SLOT(selectionChanged()));

    mainLayout = new QVBoxLayout;
    mainLayout->addWidget(carList);
    mainLayout->addWidget(buttonBox);
    filllist();

    setLayout(mainLayout);
    setWindowTitle(tr("Select Car"));
}

CarDialog::~CarDialog(){
	delete carList;
	delete buttonBox;
	delete mainLayout;
}

void CarDialog::filllist(){
	auto carvec = a->getResults();
	for (auto const& car : carvec) {
		new QListWidgetItem(QString::fromStdString(car->getModel()), carList);
	}
	carList->setCurrentRow(a->getSelectedCar());
	
}

void CarDialog::save(){
	a->setSelectedCar(carList->currentRow());
	emit saveCar(a);
	a->getString();
	close();
}

void CarDialog::discard(){
	close();
}
void CarDialog::selectionChanged(){
	std::cout<<carList->currentRow()<<std::endl;
	emit changeCar(a,carList->currentRow());
}

void CarDialog::closeEvent(QCloseEvent *event){
	emit windowClosed();
}