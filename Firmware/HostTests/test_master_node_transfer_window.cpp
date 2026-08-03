#include <cassert>
#include <iostream>

#include "MASTER_NODE_TRANSFER_WINDOW.h"

using exo::MasterNodeTransferWindow;
using exo::NodeTransferDecision;

int main()
{
    MasterNodeTransferWindow window;
    assert(!window.begin(0U, 1U, 360U, 180U));
    assert(window.begin(1U, 42U, 400U, 180U));

    auto first = window.inspect(1U, 42U, 0U, 0U, 180U, true, false);
    assert(first.decision == NodeTransferDecision::Accept);
    assert(window.commit(first));

    auto duplicate = window.inspect(1U, 42U, 0U, 0U, 180U, true, false);
    assert(duplicate.decision == NodeTransferDecision::Duplicate);

    auto gap = window.inspect(1U, 42U, 2U, 360U, 40U, true, true);
    assert(gap.decision == NodeTransferDecision::NackGap);
    assert(gap.request_chunk == 1U);

    auto corrupt = window.inspect(1U, 42U, 1U, 180U, 180U, false, false);
    assert(corrupt.decision == NodeTransferDecision::NackCorrupt);

    auto second = window.inspect(1U, 42U, 1U, 180U, 180U, true, false);
    assert(second.decision == NodeTransferDecision::Accept);
    assert(window.commit(second));

    auto final_chunk = window.inspect(1U, 42U, 2U, 360U, 40U, true, true);
    assert(final_chunk.decision == NodeTransferDecision::Complete);
    assert(window.commit(final_chunk));
    assert(window.complete());

    window.reset();
    assert(!window.active());
    std::cout << "master node transfer window tests passed\n";
    return 0;
}
