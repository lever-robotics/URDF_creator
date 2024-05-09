import React, { useContext } from 'react';
import { URDFContext } from '../URDFContext/URDFContext';

const ControlsPanel = () => {
    const { selectedPart } = useContext(URDFContext);

    if (!selectedPart) return <div>No part selected</div>;

    return (
        <div>
            <h2>Selected Part</h2>
            <p>Name: {selectedPart.name}</p>
            <p>Length: {selectedPart.parameters.length}</p>
            <p>Width: {selectedPart.parameters.width}</p>
        </div>
    );
};

export default ControlsPanel;