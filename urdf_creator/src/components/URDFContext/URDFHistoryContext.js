import React, { createContext, useState, useRef } from 'react';

export const URDFHistoryContext = createContext();

export const URDFHistoryProvider = ({ children }) => {
    const [history, setHistory] = useState([]);
    const [currentIndex, setCurrentIndex] = useState(-1);

    const guiRef = useRef();
    const codeRef = useRef();

    const addHistory = (urdfString) => {
        const newHistory = history.slice(0, currentIndex + 1);
        newHistory.push(urdfString);
        setHistory(newHistory);
        setCurrentIndex(newHistory.length - 1);
    };

    const undo = () => {
        if (currentIndex > 0) {
            const previousState = history[currentIndex - 1];
            setCurrentIndex(currentIndex - 1);
            guiRef.current.updateURDFScene(previousState);
            codeRef.current.updateURDFCode(previousState);
        }
    };

    const redo = () => {
        if (currentIndex < history.length - 1) {
            const nextState = history[currentIndex + 1];
            setCurrentIndex(currentIndex + 1);
            guiRef.current.updateURDFScene(nextState);
            codeRef.current.updateURDFCode(nextState);
        }
    };

    const updateFromGUI = (newURDF) => {
        codeRef.current.updateURDFCode(newURDF);
        addHistory(newURDF);
    };

    const updateFromCode = (newURDF) => {
        guiRef.current.updateURDFScene(newURDF);
        addHistory(newURDF);
    };

    return (
        <URDFHistoryContext.Provider value={{ history, currentIndex, addHistory, undo, redo, updateFromGUI, updateFromCode, guiRef, codeRef }}>
            {children}
        </URDFHistoryContext.Provider>
    );
};
