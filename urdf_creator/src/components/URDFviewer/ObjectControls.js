import React from 'react';

const ObjectControls = ({ deselectObject }) => {
    return (
        <div style={{ display: 'flex', flexDirection: 'column', padding: 10 }}>
            <button onClick={deselectObject}>Deselect</button>
        </div>
    );
};

export default ObjectControls;
