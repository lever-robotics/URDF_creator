// URDFHistoryProvider.js
import React, { createContext, useState, useRef } from 'react';
import { Link } from './LinkClass';

export const URDFHistoryContext = createContext();

export const URDFHistoryProvider = ({ children }) => {
    const [history, setHistory] = useState([]);
    const [currentIndex, setCurrentIndex] = useState(-1);

    const guiRef = useRef();
    const codeRef = useRef();

    const addHistory = (urdfTree) => {
        const newHistory = history.slice(0, currentIndex + 1);
        newHistory.push(urdfTree);
        setHistory(newHistory);
        setCurrentIndex(newHistory.length - 1);
    };

    const undo = () => {
        if (currentIndex > 0) {
            const previousState = history[currentIndex - 1];
            setCurrentIndex(currentIndex - 1);
            guiRef.current.updateURDFTree(previousState, false);
            codeRef.current.updateURDFTree(previousState, false);
        }
    };

    const redo = () => {
        if (currentIndex < history.length - 1) {
            const nextState = history[currentIndex + 1];
            setCurrentIndex(currentIndex + 1);
            guiRef.current.updateURDFTree(nextState, false);
            codeRef.current.updateURDFTree(nextState, false);
        }
    };

    const updateFromGUI = (newTree) => {
        guiRef.current.updateURDFTree(newTree, false);
        codeRef.current.updateURDFTree(newTree, false);
        addHistory(newTree);
    };

    const updateFromCode = (newTree) => {
        guiRef.current.updateURDFTree(newTree, false);
        codeRef.current.updateURDFTree(newTree, false);
        addHistory(newTree);
    };

    return (
        <URDFHistoryContext.Provider value={{ addHistory, undo, redo, updateFromGUI, updateFromCode }}>
            {children}
        </URDFHistoryContext.Provider>
    );
};
