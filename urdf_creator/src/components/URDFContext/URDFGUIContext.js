// URDFGUIProvider.js
import React, { createContext, useContext, useState, useImperativeHandle } from 'react';
import { Link } from './Link';
import { URDFHistoryContext } from './URDFHistoryProvider';

export const URDFGUIContext = createContext();

export const URDFGUIProvider = ({ children }) => {
    const [currentURDFTree, setCurrentURDFTree] = useState(new Link('root', { isBaseLink: true }));
    const historyContext = useContext(URDFHistoryContext);

    // Updates the current state of the URDF tree in the GUI context only
    const updateURDFTree = (newTree) => {
        setCurrentURDFTree(newTree);
    };

    // Saves the current URDF tree state to the history and updates the code provider
    const saveURDFTree = (newTree) => {
        setCurrentURDFTree(newTree);
        historyContext.updateFromGUI(newTree);
    };

    useImperativeHandle(historyContext.guiRef, () => ({
        updateURDFTree
    }));

    return (
        <URDFGUIContext.Provider value={{ currentURDFTree, updateURDFTree, saveURDFTree }}>
            {children}
        </URDFGUIContext.Provider>
    );
};
