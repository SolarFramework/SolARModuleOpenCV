# Author(s) : Loic Touraine, Stephane Leduc

unix {
        USERHOMEFOLDER = $$(HOME)
}

win32 {
    USERHOMEFOLDER = $$clean_path($$(USERPROFILE))
    isEmpty(USERHOMEFOLDER) {
        USERHOMEFOLDER = $$clean_path($$(HOMEDRIVE)$$(HOMEPATH))
    }
}

REMAKEN_RULES_ROOT = $$clean_path($$(REMAKENRULESROOT))
isEmpty(REMAKEN_RULES_ROOT) {
    REMAKEN_RULES_ROOT=$${USERHOMEFOLDER}/.remaken/rules/qmake
}
!exists($${REMAKEN_RULES_ROOT}) {
    error("Unable to locate remaken rules in " $${REMAKEN_RULES_ROOT} ". Either check your remaken installation, or provide the path to your remaken qmake rules in REMAKENRULESROOT environment variable.")
}
