#include <QtWidgets>

#include "CarDialog.h"



CarDialog::CarDialog(std::shared_ptr<AutoAuto> a_, QWidget* parent,bool more) : a(a_),QDialog(parent){
	
	this->setAttribute(Qt::WA_DeleteOnClose);
	carList = new CarDialogList(this);

	buttonBox = new QDialogButtonBox(QDialogButtonBox::Save
                                     | QDialogButtonBox::Cancel);

	if(more)buttonBox->addButton("&more",QDialogButtonBox::HelpRole);

    connect(buttonBox, SIGNAL(accepted()), this, SLOT(save()));
    connect(buttonBox, SIGNAL(rejected()), this, SLOT(discard()));
    connect(buttonBox, SIGNAL(helpRequested()), this, SLOT(continue_()));
    connect(carList, SIGNAL(itemSelectionChanged()), this, SLOT(selectionChanged()));

    mainLayout = new QVBoxLayout;
    mainLayout->addWidget(carList);
    mainLayout->addWidget(buttonBox);
    filllist();

	
    setLayout(mainLayout);

    auto parentpos= parent->mapToGlobal(QPoint(parent->frameGeometry().x(),frameGeometry().y()));

    setWindowTitle(tr("Select Car"));
    this->adjustSize();
    this->move(parentpos.x()+parent->frameGeometry().width()-this->frameGeometry().width()+20,parentpos.y()+parent->frameGeometry().height()-this->frameGeometry().height()+20);

}

CarDialog::~CarDialog(){
	delete carList;
	delete buttonBox;
	delete mainLayout;
}

CarDialogList::CarDialogList(QWidget * parent){
}

void CarDialogList::keyPressEvent(QKeyEvent *event){
	event->ignore();
}

void CarDialog::keyPressEvent(QKeyEvent * event){
	switch (event->key()){
    case Qt::Key_W:
    case Qt::Key_Up:
        if(carList->currentRow()>0)
        	carList->setCurrentRow(carList->currentRow()-1); 
        break;
    case Qt::Key_S:
    case Qt::Key_Down:
    	if(carList->currentRow()<carList->count()-1)
    		carList->setCurrentRow(carList->currentRow()+1); 
    	break;
    case Qt::Key_Escape:
    	reject();
    case Qt::Key_Enter:
    case Qt::Key_Return:
    	save();
    default:
    	event->ignore(); break;
    }
}


void CarDialog::reject() {
	std::cout << "reject()"<<std::endl;
	discard();
}

void CarDialog::filllist(){
	auto carvec = a->getResults();
	for (auto const& car : carvec) {
		new QListWidgetItem(QString::fromStdString(car->getModel()), carList);
	}
	carList->setCurrentRow(a->getSelectedCar());
	
}

void CarDialog::save(){
	success=true;
	a->setSelectedCar(carList->currentRow());
	emit saveCar(a);
	//a->getString();
	close();
	disconn();
}

void CarDialog::discard(){
	
	close();
	disconn();
}

void CarDialog::continue_(){
	success=true;
	emit continueCar(a);
	close();
}
void CarDialog::selectionChanged(){
	//std::cout<<carList->currentRow()<<std::endl;
	emit changeCar(a,carList->currentRow());
}

void CarDialog::closeEvent(QCloseEvent *event){
	if (!success){
		std::cout<<"DELETE CAR!"<<std::endl;
		emit discardCar(a);
	}
	emit windowClosed();
}

void CarDialog::viewChanged(){
	emit changeCar(a,carList->currentRow());
}

void CarDialog::disconn(){
	disconnect(this, 0, 0, 0);
	disconnect(this);
}