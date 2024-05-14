import React, { createContext, useContext, useState, useImperativeHandle } from 'react';
import { URDFHistoryContext } from './URDFHistoryContext';
import { isValidURDF } from '../../utils/URDFValidator';

export const URDFCodeContext = createContext();

export const URDFCodeProvider = ({ children }) => {
    const [currentURDFCode, setCurrentURDFCode] = useState('');
    const [isCodeValid, setIsCodeValid] = useState(true);
    const historyContext = useContext(URDFHistoryContext);

    // Updates the current state of the URDF code in the Code context only
    const updateURDFCode = (newCode) => {
        console.log("Updating URDF code");
        console.log(isValidURDF(newCode));
        setIsCodeValid(isValidURDF(newCode));
        console.log("isCodeValid");
        setCurrentURDFCode(newCode);
    };

    // Save the current URDF code and update the history and GUI context
    const saveURDFCode = () => {
        if (isCodeValid) {
            historyContext.updateFromCode(currentURDFCode);
        }
    };

    // Initialize or synchronize with the URDF from history context
    useImperativeHandle(historyContext.codeRef, () => ({
        updateURDFCode: (newCode) => {
            updateURDFCode(newCode);
        }
    }));

    return (
        <URDFCodeContext.Provider value={{ currentURDFCode, updateURDFCode, saveURDFCode, isCodeValid }}>
            {children}
        </URDFCodeContext.Provider>
    );
};
