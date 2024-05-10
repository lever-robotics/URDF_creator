// URDFCodeProvider.js
import React, { createContext, useContext, useState, useImperativeHandle } from 'react';
import { Link } from './Link';
import { URDFHistoryContext } from './URDFHistoryProvider';

export const URDFCodeContext = createContext();

export const URDFCodeProvider = ({ children }) => {
    const [currentURDFTree, setCurrentURDFTree] = useState(new Link('root', { isBaseLink: true }));
    const historyContext = useContext(URDFHistoryContext);

    // Updates the current state of the URDF tree in the Code context only
    const updateURDFTree = (newTree) => {
        setCurrentURDFTree(newTree);
    };

    // Saves the current URDF tree state to the history and updates the GUI provider
    const saveURDFTree = (newTree) => {
        setCurrentURDFTree(newTree);
        historyContext.updateFromCode(newTree);
    };

    useImperativeHandle(historyContext.codeRef, () => ({
        updateURDFTree
    }));

    return (
        <URDFCodeContext.Provider value={{ currentURDFTree, updateURDFTree, saveURDFTree }}>
            {children}
        </URDFCodeContext.Provider>
    );
};
