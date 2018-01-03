// ==========================================================================
// ScriptCommandFactory class
//
// FRC 4143: MARS/WARS
// ==========================================================================
// 2017-01-28 JKSalmon - Initial development
// ==========================================================================
#pragma once

#include <string>
#include <Commands/Command.h>
template class std::basic_string<char>;  //RJS not sure why

// ==========================================================================

class ScriptCommandFactory {
public:
	// Singleton
	static ScriptCommandFactory& GetInstance();

	void SetBlueprint(std::string script, bool prepare = true);
	std::unique_ptr<frc::Command> GetCommand();
	void PrepareCommand();

private:
	std::string m_script;
	std::unique_ptr<frc::Command> m_pCommand;

	// ctor and dtor are private to support Singleton
	ScriptCommandFactory();
	virtual ~ScriptCommandFactory();

	void _BuildOne();
	bool _HaveOne() const;
};

// ==========================================================================
