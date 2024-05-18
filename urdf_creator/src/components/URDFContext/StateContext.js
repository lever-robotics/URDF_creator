import React, { createContext, useReducer, useContext } from 'react';
import { ScenetoXML } from '../../utils/ScenetoXML';

// Initial state
const initialState = {
    scene: null,
    URDFCode: ''
};

// Create context
const StateContext = createContext(initialState);

// Reducer function
const stateReducer = (state, action) => {
    switch (action.type) {
        case 'SET_SCENE':
            return {
                ...state,
                scene: action.payload,
                URDFCode: ScenetoXML(action.payload),
            };
        case 'SET_URDF_CODE':
            return {
                ...state,
                URDFCode: action.payload,
                // This is where we will but scene = XMLToScene(action.payload)
            };
        default:
            return state;
    }
};

// Provider component
const StateProvider = ({ children }) => {
    const [state, dispatch] = useReducer(stateReducer, initialState);

    return (
        <StateContext.Provider value={{ state, dispatch }}>
            {children}
        </StateContext.Provider>
    );
};

// Custom hook to use the StateContext
const useStateContext = () => useContext(StateContext);

export { StateProvider, useStateContext };
