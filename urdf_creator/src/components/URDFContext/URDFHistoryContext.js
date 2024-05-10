import React, { createContext, useState, useRef } from 'react';

export const URDFHistoryContext = createContext();

export const URDFHistoryProvider = ({ children }) => {
    const [history, setHistory] = useState([]);
    const [currentIndex, setCurrentIndex] = useState(-1);

    const guiRef = useRef();
    const codeRef = useRef();

    // Add a URDF string to version control so all versions of the project are saved
    const addHistory = (urdfString) => {
        const newHistory = history.slice(0, currentIndex + 1);
        newHistory.push(urdfString);
        setHistory(newHistory);
        setCurrentIndex(newHistory.length - 1);
    };

    // Make the current URDF of the gui and code Context set to the last URDF string
    const undo = () => {
        if (currentIndex > 0) {
            const previousState = history[currentIndex - 1];
            setCurrentIndex(currentIndex - 1);
            guiRef.current.updateURDFCode(previousState);
            codeRef.current.updateURDFCode(previousState);
        }
    };

    // Make the current URDF of the gui and code Context set to the next URDF string
    const redo = () => {
        if (currentIndex < history.length - 1) {
            const nextState = history[currentIndex + 1];
            setCurrentIndex(currentIndex + 1);
            guiRef.current.updateURDFCode(nextState);
            codeRef.current.updateURDFCode(nextState);
        }
    };

    // Update the Gui Context, this will be used every time Code Editor is the first to edit the code and Save
    const updateFromGUI = (newURDF) => {
        guiRef.current.updateURDFCode(newURDF);
        codeRef.current.updateURDFCode(newURDF);
        addHistory(newURDF);
    };

    // Update the Code Context, this will be used every time Gui is the first to edit the URDF and save
    const updateFromCode = (newURDF) => {
        guiRef.current.updateURDFCode(newURDF);
        codeRef.current.updateURDFCode(newURDF);
        addHistory(newURDF);
    };

    return (
        <URDFHistoryContext.Provider value={{ addHistory, undo, redo, updateFromGUI, updateFromCode }}>
            {children}
        </URDFHistoryContext.Provider>
    );
};
