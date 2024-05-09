import React, { createContext, useState } from 'react';

export const URDFContext = createContext();

export function URDFProvider({ children }) {
    const [urdf, setUrdf] = useState(null);
    const [selectedObject, setSelectedObject] = useState(null);

    const updateURDF = newURDF => {
        setUrdf(newURDF);
    };

    const insertObject = shape => {
        setSelectedObject({ type: shape, position: { x: 0, y: 0, z: 0 } }); // Default position
    };

    const contextValue = {
        urdf,
        updateURDF,
        selectedObject,
        setSelectedObject,
        insertObject
    };

    return (
        <URDFContext.Provider value={contextValue}>
            {children}
        </URDFContext.Provider>
    );
}
