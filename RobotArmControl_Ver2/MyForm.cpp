
#include "MyForm.h"

using namespace System;
using namespace System::Windows::Forms;

void main(array<System::String^>^ args)
{
	Application::EnableVisualStyles();
	Application::SetCompatibleTextRenderingDefault(false);
	RobotArmControlVer2::MyForm form;
	Application::Run(% form);
}	

