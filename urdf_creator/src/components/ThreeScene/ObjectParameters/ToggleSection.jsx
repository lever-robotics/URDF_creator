import React, { useState } from 'react';

const ToggleSection = ({ title, children }) => {
    const [isOpen, setIsOpen] = useState(false);

    return (
        <div className="toggle-section">
            <div className="toggle-header" onClick={() => setIsOpen(!isOpen)}>
                <h4>{title}</h4>
                <button>{isOpen ? "-" : "+"}</button>
            </div>
            {isOpen && <div className="toggle-content">{children}</div>}
        </div>
    );
};

export default ToggleSection;
